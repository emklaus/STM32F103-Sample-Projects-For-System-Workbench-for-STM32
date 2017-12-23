/***************************************************************************
 * STM32 VGA demo
 * Copyright (C) 2012 Artekit Italy
 * http://www.artekit.eu
 * Written by Ruben H. Meleca
 
### gdi.h
 
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

***************************************************************************/

#ifndef	__GDI_H
#define	__GDI_H

#include "stm32f10x.h"

//	System font

#define	GDI_SYSFONT_WIDTH	6	// Width in pixels
#define	GDI_SYSFONT_HEIGHT	9	// Height in pixels
#define	GDI_SYSFONT_BYTEWIDTH	1	// Width in bytes
#define	GDI_SYSFONT_OFFSET	0x20

//	RASTER OPERATION

#define	GDI_ROP_COPY  0
#define	GDI_ROP_XOR	  1
#define	GDI_ROP_AND	  2
#define	GDI_ROP_OR	  3


typedef  struct
{
	uint16_t x;		// X position
	uint16_t y;		// Y position
	uint16_t w;		// Width
	uint16_t h;		// Height
}GDI_RECT, *PGDI_RECT;

#define	GDI_WINCAPTION		0x0001
#define	GDI_WINBORDER		0x0002
#define	GDI_WINCLOSEICON	0x0003

//	Text align mode

#define	GDI_WINCAPTION_LEFT	0x0000
#define	GDI_WINCAPTION_CENTER	0x0010
#define	GDI_WINCAPTION_RIGHT	0x0020
#define	GDI_WINCAPTION_MASK	0x0030

typedef struct
{
	uint16_t	style;		// Mode, see GDI_WINxxx defines
	GDI_RECT	rc;		// Absolute rectangle
	uint8_t	  	*caption;	// Caption text

} GDI_WINDOW, *PGDI_WINDOW;

typedef struct
{
	uint16_t	w;	// Width in bits
	uint16_t	h;	// Height in bits
	uint16_t	wb;	// width in bytes
	uint16_t	wh;	// Height in bytes
	uint8_t	        *bm;	// Pointer to bitmap bits

} GDI_BITMAP, PGDI_BITMAP;

//	Function definitions

void	gdiGetClientRect(PGDI_WINDOW, PGDI_RECT);
void	gdiCopyRect(PGDI_RECT rc1, PGDI_RECT rc2);
void	gdiBitBlt(PGDI_RECT prc, int x, int y, int w, int h, uint8_t *bm, uint8_t rop);
void	gdiPoint(PGDI_RECT rc, uint16_t x, uint16_t y, uint8_t rop);
void	gdiLine(PGDI_RECT prc, int x1, int y1, int x2, int y2, uint8_t rop);
void	gdiRectangle(int x0, int y0, int x1, int y1, uint8_t rop);
void	gdiRectangleEx(PGDI_RECT rc, uint8_t rop);
void	gdiCircle(int x, int y, int r, uint8_t rop);
void	gdiDrawWindow(PGDI_WINDOW pwin);
void	gdiDrawTextEx(int x, int y, char *ptext, uint8_t rop);

#endif	// __GDI_H
