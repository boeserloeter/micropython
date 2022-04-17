/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Damien P. George
 * Copyright (c) 2022 Jan-Hinnerk Dumjahn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * This is a naive implementation of DMX node (based on machine_uart.c)
 * Known issues
 * - it might be better design to write a generic RS485 driver with support for handling break conditions on RX
 * - there can be glitches when combining channels (e.g. to 16-bit values)
 * - there is no way to detect a communication timeout
 * - there is no way to detect how many channels have been received
 * - deinit might not work properly
 * - no support for RDM
 */

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/ringbuf.h"
#include "modmachine.h"

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/regs/uart.h"
#include "pico/mutex.h"

#define DMX_BAUDRATE (250000)
#define DMX_BITS (8)
#define DMX_PARITY (UART_PARITY_NONE)
#define DMX_STOP (1)

// UART 0 default pins
#if !defined(MICROPY_HW_UART0_TX)
#define MICROPY_HW_UART0_TX (0)
#define MICROPY_HW_UART0_RX (1)
#endif

// UART 1 default pins
#if !defined(MICROPY_HW_UART1_TX)
#define MICROPY_HW_UART1_TX (4)
#define MICROPY_HW_UART1_RX (5)
#endif

#define MAX_DMX_CHANNELS (512)

#define IS_VALID_PERIPH(uart, pin)  (((((pin) + 4) & 8) >> 3) == (uart))
#define IS_VALID_TX(uart, pin)      (((pin) & 3) == 0 && IS_VALID_PERIPH(uart, pin))
#define IS_VALID_RX(uart, pin)      (((pin) & 3) == 1 && IS_VALID_PERIPH(uart, pin))

#define UART_INVERT_TX (1)
#define UART_INVERT_RX (2)
#define UART_INVERT_MASK (UART_INVERT_TX | UART_INVERT_RX)


typedef struct _machine_uart_obj_t {
    mp_obj_base_t base;
    uart_inst_t *const uart;
    uint8_t uart_id;
    uint8_t tx;
    uint8_t rx;
    uint8_t invert;
    int rx_index;
    uint32_t rx_count;      // for debugging
    uint32_t break_count;   // for debugging
    uint8_t rx_buffer[MAX_DMX_CHANNELS];
} machine_uart_obj_t;

STATIC machine_uart_obj_t machine_uart_obj[] = {
    {{&machine_uart_dmx_type}, uart0, 0,
     MICROPY_HW_UART0_TX, MICROPY_HW_UART0_RX,
     0, -2, 0, 0, {}},
    {{&machine_uart_dmx_type}, uart1, 1,
     MICROPY_HW_UART1_TX, MICROPY_HW_UART1_RX,
     0, -2, 0, 0, {}},
};

STATIC const char *_invert_name[] = {"None", "INV_TX", "INV_RX", "INV_TX|INV_RX"};

/******************************************************************************/
// IRQ and buffer handling

// take all bytes from the fifo and store them in the buffer
STATIC void uart_drain_rx_fifo(machine_uart_obj_t *self) {
    while (uart_is_readable(self->uart)) {
        // get a byte from uart and put into the buffer
        const uint16_t x = uart_get_hw(self->uart)->dr;
        ++self->rx_count;
        if(x & 0x0400) {        // handle break
            self->rx_index = -1;
            ++self->break_count;
        }
        else if(self->rx_index == -1) {
            if (x == 0) {
                self->rx_index = 0;
            }
            else {
                self->rx_index = -2;
            }
        }
        else if(self->rx_index >= 0 && self->rx_index < MAX_DMX_CHANNELS) {
            self->rx_buffer[self->rx_index] = x;
            ++self->rx_index;
        }
    }
}

STATIC inline void uart_service_interrupt(machine_uart_obj_t *self) {
    if (uart_get_hw(self->uart)->mis & (UART_UARTMIS_RXMIS_BITS | UART_UARTMIS_RTMIS_BITS)) { // rx interrupt?
        // clear all interrupt bits but tx
        uart_get_hw(self->uart)->icr = UART_UARTICR_BITS & (~UART_UARTICR_TXIC_BITS);
        uart_drain_rx_fifo(self);
    }
    if (uart_get_hw(self->uart)->mis & UART_UARTMIS_TXMIS_BITS) { // tx interrupt?
        // clear all interrupt bits but rx
        uart_get_hw(self->uart)->icr = UART_UARTICR_BITS & (~UART_UARTICR_RXIC_BITS);
    }
}

STATIC void uart0_irq_handler(void) {
    uart_service_interrupt(&machine_uart_obj[0]);
}

STATIC void uart1_irq_handler(void) {
    uart_service_interrupt(&machine_uart_obj[1]);
}

/******************************************************************************/
// MicroPython bindings for UART

STATIC void machine_uart_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "UARTDMX(%u, tx=%d, rx=%d, invert=%s, "
        "rx_index=%u, rx_count=%u, brk_count=%u)",
        self->uart_id, self->tx, self->rx, _invert_name[self->invert],
        self->rx_index, self->rx_count, self->break_count);
}

STATIC void machine_uart_init_helper(machine_uart_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_tx, ARG_rx, ARG_invert};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_tx, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_rx, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_invert, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);


    // Set TX/RX pins if configured.
    if (args[ARG_tx].u_obj != mp_const_none) {
        int tx = mp_hal_get_pin_obj(args[ARG_tx].u_obj);
        if (!IS_VALID_TX(self->uart_id, tx)) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad TX pin"));
        }
        self->tx = tx;
    }
    if (args[ARG_rx].u_obj != mp_const_none) {
        int rx = mp_hal_get_pin_obj(args[ARG_rx].u_obj);
        if (!IS_VALID_RX(self->uart_id, rx)) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad RX pin"));
        }
        self->rx = rx;
    }

    // Set line inversion if configured.
    if (args[ARG_invert].u_int >= 0) {
        if (args[ARG_invert].u_int & ~UART_INVERT_MASK) {
            mp_raise_ValueError(MP_ERROR_TEXT("bad inversion mask"));
        }
        self->invert = args[ARG_invert].u_int;
    }

    // Initialise the UART peripheral

    uart_init(self->uart, DMX_BAUDRATE);
    uart_set_format(self->uart, DMX_BITS, DMX_STOP, DMX_PARITY);
    uart_set_fifo_enabled(self->uart, true);
//    gpio_set_function(self->tx, GPIO_FUNC_UART);
    gpio_set_function(self->rx, GPIO_FUNC_UART);
    if (self->invert & UART_INVERT_RX) {
        gpio_set_inover(self->rx, GPIO_OVERRIDE_INVERT);
    }
//    if (self->invert & UART_INVERT_TX) {
//        gpio_set_outover(self->tx, GPIO_OVERRIDE_INVERT);
//    }

    uart_set_hw_flow(self->uart, false, false);


    // Set the irq handler.
    if (self->uart_id == 0) {
        irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
        irq_set_enabled(UART0_IRQ, true);
    } else {
        irq_set_exclusive_handler(UART1_IRQ, uart1_irq_handler);
        irq_set_enabled(UART1_IRQ, true);
    }

    // Enable the uart irq; this macro sets the rx irq level to 4.
    uart_set_irq_enables(self->uart, true, false);      // only enable RX
}

STATIC mp_obj_t machine_uart_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // Get UART bus.
    int uart_id = mp_obj_get_int(args[0]);
    if (uart_id < 0 || uart_id >= MP_ARRAY_SIZE(machine_uart_obj)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("UART(%d) doesn't exist"), uart_id);
    }

    // Get static peripheral object.
    machine_uart_obj_t *self = (machine_uart_obj_t *)&machine_uart_obj[uart_id];

    // Initialise the UART peripheral.
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    machine_uart_init_helper(self, n_args - 1, args + 1, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_uart_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    // Initialise the UART peripheral.
    machine_uart_init_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(machine_uart_dmx_init_obj, 1, machine_uart_init);

STATIC mp_obj_t machine_uart_deinit(mp_obj_t self_in) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uart_deinit(self->uart);
    if (self->uart_id == 0) {
        irq_set_enabled(UART0_IRQ, false);
    } else {
        irq_set_enabled(UART1_IRQ, false);
    }
    MP_STATE_PORT(rp2_uart_rx_buffer[self->uart_id]) = NULL;
    MP_STATE_PORT(rp2_uart_tx_buffer[self->uart_id]) = NULL;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_uart_deinit_obj, machine_uart_deinit);


STATIC const mp_rom_map_elem_t machine_uart_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_uart_dmx_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_uart_deinit_obj) },

    { MP_ROM_QSTR(MP_QSTR_INV_TX), MP_ROM_INT(UART_INVERT_TX) },
    { MP_ROM_QSTR(MP_QSTR_INV_RX), MP_ROM_INT(UART_INVERT_RX) },

};
STATIC MP_DEFINE_CONST_DICT(machine_uart_locals_dict, machine_uart_locals_dict_table);

STATIC mp_obj_t machine_uart_dmx_subscr(mp_obj_t self_in, mp_obj_t index, mp_obj_t value)
{
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (value == MP_OBJ_SENTINEL) {
        // load
        if (mp_obj_is_type(index, &mp_type_slice)) {
            mp_bound_slice_t slice;
            if (!mp_seq_get_fast_slice_indexes(MAX_DMX_CHANNELS, index, &slice)) {
                mp_raise_NotImplementedError(MP_ERROR_TEXT("only slices with step=1 (aka None) are supported"));
            }
            return mp_obj_new_bytes(self->rx_buffer + slice.start, slice.stop - slice.start);
        }
        else if(mp_obj_is_small_int(index)) {
            const mp_int_t i = MP_OBJ_SMALL_INT_VALUE(index);
            if(i >=0 && i < MAX_DMX_CHANNELS) {
                return MP_OBJ_NEW_SMALL_INT(self->rx_buffer[i]);
            }
        }
        mp_raise_msg(&mp_type_IndexError, MP_ERROR_TEXT("DMX channel out of range"));
    }

    return MP_OBJ_NULL;
}

const mp_obj_type_t machine_uart_dmx_type = {
    { &mp_type_type },
    .name = MP_QSTR_UARTDMX,
    .print = machine_uart_print,
    .make_new = machine_uart_make_new,
    .subscr = machine_uart_dmx_subscr,
    .locals_dict = (mp_obj_dict_t *)&machine_uart_locals_dict,
};
