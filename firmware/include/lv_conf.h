/**
 * @file lv_conf.h
 * @brief LVGL Configuration for SenseCAP Indicator
 *
 * This file configures LVGL for the 480x480 ST7701 RGB display.
 * Compatible with SquareLine Studio generated UI code.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/* Color depth: 16 (RGB565) */
#define LV_COLOR_DEPTH 16

/* Swap the 2 bytes of RGB565 color - disabled */
#define LV_COLOR_16_SWAP 0

/* Enable more complex drawing routines to manage masks */
#define LV_DRAW_COMPLEX 1

/* Default image cache size. Image caching keeps the images opened. */
#define LV_IMG_CACHE_DEF_SIZE 0

/* Number of gradient stops */
#define LV_GRADIENT_MAX_STOPS 2

/*====================
   MEMORY SETTINGS
 *====================*/

/* Size of the memory available for lv_mem_alloc() in bytes */
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (48U * 1024U)  /* 48KB */
#define LV_MEM_ADR 0
#define LV_MEM_BUF_MAX_NUM 16

/* Use the standard memcpy and memset instead of LVGL's own functions */
#define LV_MEMCPY_MEMSET_STD 1

/*====================
   HAL SETTINGS
 *====================*/

/* Default display refresh period in milliseconds */
#define LV_DISP_DEF_REFR_PERIOD 16  /* ~60 fps */

/* Input device read period in milliseconds */
#define LV_INDEV_DEF_READ_PERIOD 30

/* Use a custom tick source */
#define LV_TICK_CUSTOM 0

/* Default Dots Per Inch */
#define LV_DPI_DEF 130

/*====================
   FEATURE CONFIGURATION
 *====================*/

/* Drawing engine */
#define LV_DRAW_SW_COMPLEX 1
#define LV_DRAW_SW_SHADOW_CACHE_SIZE 0
#define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4

/* GPU */
#define LV_USE_GPU_STM32_DMA2D 0
#define LV_USE_GPU_NXP_PXP 0
#define LV_USE_GPU_NXP_VG_LITE 0
#define LV_USE_GPU_SDL 0

/*====================
   LOGGING
 *====================*/

/* Enable the log module */
#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN

/* Print the log with 'printf' */
#define LV_LOG_PRINTF 1

/* Enable/disable LV_LOG_TRACE in modules */
#define LV_LOG_TRACE_MEM 0
#define LV_LOG_TRACE_TIMER 0
#define LV_LOG_TRACE_INDEV 0
#define LV_LOG_TRACE_DISP_REFR 0
#define LV_LOG_TRACE_EVENT 0
#define LV_LOG_TRACE_OBJ_CREATE 0
#define LV_LOG_TRACE_LAYOUT 0
#define LV_LOG_TRACE_ANIM 0

/*====================
   ASSERTS
 *====================*/

#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ 0

#define LV_ASSERT_HANDLER_INCLUDE <stdint.h>
#define LV_ASSERT_HANDLER while(1);

/*====================
   OTHERS
 *====================*/

/* Show CPU and FPS usage - disabled to avoid garbled text */
#define LV_USE_PERF_MONITOR 0
#define LV_USE_PERF_MONITOR_POS LV_ALIGN_BOTTOM_RIGHT

/* Show memory usage */
#define LV_USE_MEM_MONITOR 0

/* Sprintf for LVGL */
#define LV_SPRINTF_CUSTOM 0
#define LV_SPRINTF_USE_FLOAT 0

/* Compiler prefix for a big array declaration in RAM */
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY

/* Prefix for function pointers */
#define LV_FUNC_ATTR

/* Export integer constant to binding */
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning

/* Prefix variables that are used in GPU accelerated operations */
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 1
#define LV_ATTRIBUTE_MEM_ALIGN

/* The buffer is placed to a special memory region that can be accessed
 * by GPU accelerator */
#define LV_ATTRIBUTE_DMA

/* Garbage Collector settings */
#define LV_GC_INCLUDE "lv_gc.h"
#define LV_ENABLE_GC 0

/*====================
   FONT USAGE
 *====================*/

/* Montserrat fonts with ASCII range and some symbols */
#define LV_FONT_MONTSERRAT_8 1
#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_30 1
#define LV_FONT_MONTSERRAT_32 1
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

/* Demonstrate special features */
#define LV_FONT_MONTSERRAT_12_SUBPX 0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_16_CJK 0

/* Pixel perfect monospace fonts */
#define LV_FONT_UNSCII_8 0
#define LV_FONT_UNSCII_16 0

/* Optionally declare custom fonts here */
#define LV_FONT_CUSTOM_DECLARE

/* Default font */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/* Enable handling large font and/or fonts with a lot of characters */
#define LV_FONT_FMT_TXT_LARGE 0

/* Enables/disables support for compressed fonts */
#define LV_USE_FONT_COMPRESSED 0

/* Enable subpixel rendering */
#define LV_USE_FONT_SUBPX 0

/*====================
   TEXT SETTINGS
 *====================*/

/* Select a character encoding for strings */
#define LV_TXT_ENC LV_TXT_ENC_UTF8

/* Can break (wrap) texts on these chars */
#define LV_TXT_BREAK_CHARS " ,.;:-_"

/* If a word is at least this long, will break wherever fits */
#define LV_TXT_LINE_BREAK_LONG_LEN 0

/* Minimum number of characters in a long word to put on a line before a break */
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3

/* Minimum number of characters in a long word to put on a line after a break */
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

/* The control character to use for signalling text recoloring */
#define LV_TXT_COLOR_CMD "#"

/* Support bidirectional texts */
#define LV_USE_BIDI 0

/* Enable Arabic/Persian processing */
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*====================
   WIDGET USAGE
 *====================*/

/* Base widget */
#define LV_USE_ARC 1
#define LV_USE_BAR 1
#define LV_USE_BTN 1
#define LV_USE_BTNMATRIX 1
#define LV_USE_CANVAS 1
#define LV_USE_CHECKBOX 1
#define LV_USE_DROPDOWN 1
#define LV_USE_IMG 1
#define LV_USE_LABEL 1
#define LV_LABEL_TEXT_SELECTION 1
#define LV_LABEL_LONG_TXT_HINT 1
#define LV_USE_LINE 1
#define LV_USE_ROLLER 1
#define LV_ROLLER_INF_PAGES 7
#define LV_USE_SLIDER 1
#define LV_USE_SWITCH 1
#define LV_USE_TEXTAREA 1
#define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500
#define LV_USE_TABLE 1

/*====================
   EXTRA COMPONENTS
 *====================*/

/* Widgets */
#define LV_USE_ANIMIMG 1
#define LV_USE_CALENDAR 1
#define LV_CALENDAR_WEEK_STARTS_MONDAY 0
#define LV_USE_CALENDAR_HEADER_ARROW 1
#define LV_USE_CALENDAR_HEADER_DROPDOWN 1
#define LV_USE_CHART 1
#define LV_USE_COLORWHEEL 1
#define LV_USE_IMGBTN 1
#define LV_USE_KEYBOARD 1
#define LV_USE_LED 1
#define LV_USE_LIST 1
#define LV_USE_MENU 1
#define LV_USE_METER 1
#define LV_USE_MSGBOX 1
#define LV_USE_SPAN 1
#define LV_SPAN_SNIPPET_STACK_SIZE 64
#define LV_USE_SPINBOX 1
#define LV_USE_SPINNER 1
#define LV_USE_TABVIEW 1
#define LV_USE_TILEVIEW 1
#define LV_USE_WIN 1

/* Themes */
#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1
#define LV_THEME_DEFAULT_GROW 1
#define LV_THEME_DEFAULT_TRANSITION_TIME 80
#define LV_USE_THEME_BASIC 1
#define LV_USE_THEME_MONO 1

/* Layouts */
#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/* Others */
#define LV_USE_SNAPSHOT 1
#define LV_USE_MONKEY 0
#define LV_USE_GRIDNAV 1
#define LV_USE_FRAGMENT 0
#define LV_USE_IMGFONT 0
#define LV_USE_MSG 1
#define LV_USE_IME_PINYIN 0

/* File system interfaces */
#define LV_USE_FS_STDIO 0
#define LV_USE_FS_POSIX 0
#define LV_USE_FS_WIN32 0
#define LV_USE_FS_FATFS 0

/* PNG decoder library */
#define LV_USE_PNG 0

/* BMP decoder library */
#define LV_USE_BMP 0

/* JPG + split JPG decoder library */
#define LV_USE_SJPG 0

/* GIF decoder library */
#define LV_USE_GIF 0

/* QR code library */
#define LV_USE_QRCODE 0

/* FreeType library */
#define LV_USE_FREETYPE 0

/* Tiny TTF library */
#define LV_USE_TINY_TTF 0

/* Rlottie library */
#define LV_USE_RLOTTIE 0

/* FFmpeg library for image decoding and playing videos */
#define LV_USE_FFMPEG 0

#endif /* LV_CONF_H */
