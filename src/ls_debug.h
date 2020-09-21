/*
 * Copyright (C) 2020 LeakSentinel, Inc. or its affiliates.  All Rights Reserved.
 *
 */

/* global defines for LeakSentinel firmware */

#ifndef _LS_DEBUG_
#define _LS_DEBUG_

/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8    "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8( i ) \
    ( ( ( i ) & 0x80ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x40ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x20ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x10ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x08ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x04ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x02ll ) ? '1' : '0' ), \
    ( ( ( i ) & 0x01ll ) ? '1' : '0' )

#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8 PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16( i ) \
    PRINTF_BYTE_TO_BINARY_INT8( ( i ) >> 8 ), PRINTF_BYTE_TO_BINARY_INT8( i )
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16 PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32( i ) \
    PRINTF_BYTE_TO_BINARY_INT16( ( i ) >> 16 ), PRINTF_BYTE_TO_BINARY_INT16( i )
#define PRINTF_BINARY_PATTERN_INT64 \
    PRINTF_BINARY_PATTERN_INT32 PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64( i ) \
    PRINTF_BYTE_TO_BINARY_INT32( ( i ) >> 32 ), PRINTF_BYTE_TO_BINARY_INT32( i )
/* --- end macros --- */

/*
 * printf("My Flag "
 *      PRINTF_BINARY_PATTERN_INT64 "\n",
 *      PRINTF_BYTE_TO_BINARY_INT64(flag));
 * return 0;
 */
#endif /* _LS_DEBUG_ */
