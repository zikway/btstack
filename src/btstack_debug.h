/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

/**
 * @title Debug Messages
 *
 * Allow to funnel debug and error messages.
 *
 */

#ifndef DEBUG_H
#define DEBUG_H

#include "btstack_config.h"
#include "btstack_defines.h"
#include "hci_dump.h"

#if defined __cplusplus
extern "C" {
#endif

#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

#ifdef HAVE_ASSERT
#ifndef btstack_assert
#include <assert.h>
#endif
#endif

// fallback to __FILE__ for untagged files
#ifndef BTSTACK_FILE__
#define BTSTACK_FILE__ __FILE__
#endif

#ifdef HAVE_ASSERT
// allow to override btstack_assert in btstack_config.h
#ifndef btstack_assert
// map to libc assert
#ifdef NDEBUG
#define btstack_assert(condition)  {(void)(condition);}
#else
#define btstack_assert(condition)  assert(condition)
#endif
#endif /* btstack_assert */
#else /* HAVE_ASSERT */
#ifdef ENABLE_BTSTACK_ASSERT
#include <stdnoreturn.h>
noreturn void btstack_assert_failed(const char * file, uint16_t line_nr);
#ifndef btstack_assert
// use btstack macro that calls btstack_assert_failed() - provided by port
#define btstack_assert(condition)         if (condition) {} else { btstack_assert_failed(BTSTACK_FILE__, __LINE__);  }
#endif
#else /* ENABLE_BTSTACK_ASSERT */
// asserts off
#define btstack_assert(condition)         {(void)(condition);}
#endif /* btstack_assert */
#endif /* HAVE_ASSERT */

// mark code that should not be reached. Similar to assert, but mapped to NOP for coverage
#ifdef UNIT_TEST
#define btstack_unreachable()
#else
#define btstack_unreachable() btstack_assert(false)
#endif

// allow to provide port specific printf
#ifndef BTSTACK_PRINTF
#ifdef __AVR__
#define BTSTACK_PRINTF(format, ...)          logd(format, ## __VA_ARGS__)////printf_P(PSTR(format), ## __VA_ARGS__)
#else
#define BTSTACK_PRINTF(...)          logd(__VA_ARGS__)////printf( __VA_ARGS__)
#endif
#endif

#ifdef __AVR__
#define HCI_DUMP_LOG_PRINTF(log_level, format, ...) hci_dump_log_P(log_level, PSTR("%S.%u: " format), PSTR(BTSTACK_FILE__), __LINE__, ## __VA_ARGS__)
#define HCI_DUMP_LOG_PUTS(log_level, format)        hci_dump_log_P(log_level, PSTR("%S.%u: " format), PSTR(BTSTACK_FILE__), __LINE__)
#else
#define HCI_DUMP_LOG_PRINTF(log_level, format, ...) hci_dump_log(log_level, "%s.%u: " format, BTSTACK_FILE__, __LINE__, ## __VA_ARGS__)
#define HCI_DUMP_LOG_PUTS(log_level, format)        hci_dump_log(log_level, "%s.%u: " format, BTSTACK_FILE__, __LINE__);
#endif

#ifdef _MSC_VER

// original version that requires GNU Macro extensions, but works with Visual Studio 2022

#define HCI_DUMP_LOG HCI_DUMP_LOG_PRINTF

#ifdef ENABLE_LOG_DEBUG
#define log_debug(format, ...)  logd(format, ## __VA_ARGS__)//HCI_DUMP_LOG(HCI_DUMP_LOG_LEVEL_DEBUG, format,  ## __VA_ARGS__)
#else
#define log_debug(...) (void)(0)
#endif

#ifdef ENABLE_LOG_INFO
#define log_info(format, ...)  logd(format, ## __VA_ARGS__)////HCI_DUMP_LOG(HCI_DUMP_LOG_LEVEL_INFO, format,  ## __VA_ARGS__)
#else
#define log_info(...) (void)(0)
#endif

#ifdef ENABLE_LOG_ERROR
#define log_error(format, ...)  logd(format, ## __VA_ARGS__)////HCI_DUMP_LOG(HCI_DUMP_LOG_LEVEL_ERROR, format,  ## __VA_ARGS__)
#else
#define log_error(...) (void)(0)
#endif

#else  /* _MSC_VER */

// C99 Pedantic version - does not work for Visual Studio 2022

#define GET_LOGGER_TYPE_FOR_ARG_COUNT( _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, NAME, ... ) NAME

#define HCI_DUMP_LOG( ... ) GET_LOGGER_TYPE_FOR_ARG_COUNT(__VA_ARGS__, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PRINTF, HCI_DUMP_LOG_PUTS, UNUSED)( __VA_ARGS__ )

#ifdef ENABLE_LOG_DEBUG
#define log_debug(...)  logd(__VA_ARGS__)//HCI_DUMP_LOG(HCI_DUMP_LOG_LEVEL_DEBUG, ## __VA_ARGS__)
#else
#define log_debug(...) (void)(0)
#endif

#ifdef ENABLE_LOG_INFO
#define log_info(...)  logd(__VA_ARGS__)////HCI_DUMP_LOG(HCI_DUMP_LOG_LEVEL_INFO, ## __VA_ARGS__)
#else
#define log_info(...) (void)(0)
#endif

#ifdef ENABLE_LOG_ERROR
#define log_error(...)  logd(__VA_ARGS__)//HCI_DUMP_LOG(HCI_DUMP_LOG_LEVEL_ERROR, ## __VA_ARGS__)
#else
#define log_error(...) (void)(0)
#endif

#endif /* _MSC_VER */


/* API_START */

/**
 * @brief Log Security Manager key via log_info
 * @param name
 * @param key to log
 */
void log_info_key(const char * name, sm_key_t key);

/**
 * @brief Hexdump via log_info
 * @param data
 * @param size
 */
void log_info_hexdump(const void *data, int size);

/**
 * @brief Hexdump via log_debug
 * @param data
 * @param size
 */
void log_debug_hexdump(const void *data, int size);

/* API_END */

#if defined __cplusplus
}
#endif

#endif // DEBUG_H
