/***************************************************************************
 * STM32 VGA demo
 * Copyright (C) 2012 Artekit Italy
 * http://www.artekit.eu
 * Written by Ruben H. Meleca
 
### gdi.c
 
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

#include "gdi.h"
#include "mth.h"
#include "video.h"
#include "stm32f10x.h"

extern uint8_t	fb[VID_VSIZE][VID_HSIZE+2];
extern const uint8_t 	gdiSystemFont[];

const uint8_t	gdiCloseBm[] = 	{ 	0x7f, 0xC0,
									0x7f, 0xC0,
									0x7f, 0xC0,
									0x7f, 0xC0,
									0x40, 0x40,
									0x7f, 0xC0,
									0x7f, 0xC0,
									0x7f, 0xC0,
									0x7f, 0xC0 };


uint16_t	strLen(char *str) {

int		i = 0;

	while (*str != 0) {
		++i;
		++str;
	}
	return(i);
}

//*****************************************************************************
//	Function gdiCopyRect(PGDI_RECT rc1, PGDI_RECT rc2) 
//
//	Copy rectangle rc2 to rc1
//
//	parameters:
//		rc1			Destination rectangle
//		y			Source rectangle
//
//	return:			none
//*****************************************************************************
void gdiCopyRect(PGDI_RECT rc1, PGDI_RECT rc2) {

	rc1->x = rc2->x;
	rc1->y = rc2->y;
	rc1->w = rc2->w;
	rc1->h = rc2->h;
}

//*****************************************************************************
//	Function gdiBitBlt(PGDI_RECT prc, i16 x, i16 y, i16 w, i16 h, pu8 bm, u16 rop)
//
//	Bit Block Transfer funcion. This function uses the STM32 Bit-Banding mode
//	to simplify the complex BitBlt functionality.
//
//	From Cortex STM32F10x Reference Manual (RM0008):
//	A mapping formula shows how to reference each word in the alias region to a 
//	corresponding bit in the bit-band region. The mapping formula is:
//	bit_word_addr = bit_band_base + (byte_offset x 32) + (bit_number × 4)
//	where:
//	bit_word_addr is the address of the word in the alias memory region that 
//	maps to the targeted bit.
//	bit_band_base is the starting address of the alias region
//	byte_offset is the number of the byte in the bit-band region that contains 
//	the targeted bit bit_number is the bit position (0-7) of the targeted bit.
//	Example:
//	The following example shows how to map bit 2 of the byte located at SRAM 
//	address 0x20000300 in the alias region:
//	0x22006008 = 0x22000000 + (0x300*32) + (2*4).
//	Writing to address 0x22006008 has the same effect as a read-modify-write 
//	operation on bit 2 of the byte at SRAM address 0x20000300.
//	Reading address 0x22006008 returns the value (0x01 or 0x00) of bit 2 of 
//	the byte at SRAM address 0x20000300 (0x01: bit set; 0x00: bit reset).
//
//	For further reference see the Cortex M3 Technical Reference Manual
//
//	Parameters:
//
//		prc			Clipping rectangle. All X/Y coordinates are inside "prc"
//					If "prc" is NULL, the coordinates will be the entire display
//					area
//		x			Bitmap X start position
//		y			Bitmap Y start position
//		w			Bitmap width, in pixels
//		y			Bitmap height, in pixels
//		bm			Pointer to te bitmap start position
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void gdiBitBlt(PGDI_RECT prc, int x, int y, int w, int h, uint8_t *bm, uint8_t rop)
{
uint16_t	i, xz, xb, xt;
uint32_t	wb;					// Width in bytes
uint32_t	r;					// Start X position in bits (relative to x)
uint32_t	k;
uint32_t	d;
uint32_t	offs;
uint8_t		c;
uint8_t		*fbPtr;				// Pointer to the Frame Buffer Bit-Band area
uint8_t		*fbBak;
uint8_t		fb1;
uint32_t	fb2;
uint32_t	rp;
uint8_t		*bmPtr;				// Pointer to the bitmap bits

//	Calculate clipping region

	if (prc != 0) {
		x = prc->x + x;
		y = prc->y + y;
	}

//	Get total bitmap width in bytes

	wb = (uint32_t) w >> 3;
	if ((wb << 3) < (uint32_t) w) ++wb;

//	Get starting bit inside the first byte

	d = (uint32_t) x >> 3;
	r  = ((uint32_t) x - (d << 3));

//	Clip X

	if (prc == 0) {
		if ((x + w) >= VID_PIXELS_X ) {
			xt =  VID_PIXELS_X - x;
		} else {
			xt = w;
		}
	} else {
		if ((x + w) >= (x + prc->w)) {
			xt = prc->w - x;
		} else {
			xt = w;
		}
	}

//	Draw bits

	for (i = 0; i < h; i++) {

//	Clip Y

		if ((i + y) > (VID_VSIZE - 1)) return;

//	Get offset to frame buffer in bit-banding mode

		offs = (((uint32_t) x >> 3)) + ((uint32_t) (y + i)  * VID_HSIZE_R);
		k = (uint32_t) (&fb - 0x20000000);
		k += offs;
		fbPtr = (uint8_t *) (0x22000000 + (k * 32) + ((7 - r) * 4));
		fbBak = (uint8_t *) (0x22000000 + (k * 32) + 28);

//	Get offset to bitmap bits

		bmPtr = bm + ((uint32_t) i * wb);
		xz = w;

		xb = 0;
		for (xz = 0; xz < xt; xz++) {
			fb1 = ((uint32_t) fbPtr) & 0x000000E0;
			if (xb++ == 0) {
				c = *bmPtr;
				++bmPtr;
			}
			xb &= 0x07;
			(c & 0x80) ? (rp = 1) : (rp = 0);
			switch(rop) {
				case GDI_ROP_COPY:	*fbPtr = rp;		break;
				case GDI_ROP_XOR:	*fbPtr ^= rp;		break;
				case GDI_ROP_AND:	*fbPtr &= rp;		break;
				case GDI_ROP_OR:	*fbPtr |= rp;		break;
			}
			fbPtr -= 4;
			fb2 = ((uint32_t) fbPtr) & 0x000000E0;
			if (fb1 != fb2) {
				fbPtr = fbBak + 32;
				fbBak = fbPtr;
			}
			c <<= 1;
		}
	}
}

//*****************************************************************************
//	Function gdiPoint(PGDI_RECT rc, u16 x, u16 y)
//
//	Show a point in x/y position using the current graphical mode stored in 
//	grMode variable
//
//	parameters:
//		x			X position
//		y			Y position
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return:			none
//*****************************************************************************
void gdiPoint(PGDI_RECT rc, uint16_t x, uint16_t y, uint8_t rop) {

uint16_t	w, r;
uint8_t		m;

//	Test for point outside display area

	if (x >= VID_PIXELS_X || y >= VID_PIXELS_Y) return;

	w = x >> 3;
	r = x - (w << 3);

//	Prepare mask

	m = (0x80 >> r);

	switch(rop) {
		case GDI_ROP_COPY:		fb[y][w] |= m;
								break;
		case GDI_ROP_XOR:		fb[y][w] ^= m;
								break;
		case GDI_ROP_AND:		fb[y][w] &= m;
								break;
	}
}

//*****************************************************************************
//	Function gdiLine(i16 x1, i16 y1, i16 x2, i16 y2, u16 rop)
//
//	Draw line using Bresenham algorithm 
//
//	This function was taken from the book:
//	Interactive Computer Graphics, A top-down approach with OpenGL
//	written by Emeritus Edward Angel
//
//	parameters:
//		prc			Clipping rectangle
//		x1			X start position
//		y1			Y start position
//		x2			X end position
//		y2			Y end position
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void gdiLine(PGDI_RECT prc, int x1, int y1, int x2, int y2, uint8_t rop) {

int		dx, dy, i, e;
int		incx, incy, inc1, inc2;
int		x, y;

	dx = x2 - x1;
	dy = y2 - y1;

	if(dx < 0) dx = -dx;
	if(dy < 0) dy = -dy;
	incx = 1;
	if(x2 < x1) incx = -1;
	incy = 1;
	if(y2 < y1) incy = -1;
	x=x1;
	y=y1;

	if (dx > dy) {
		gdiPoint(prc, x, y, rop);
		e = 2*dy - dx;
		inc1 = 2 * ( dy -dx);
		inc2 = 2 * dy;
		for (i = 0; i < dx; i++) {
			if (e >= 0) {
				y += incy;
				e += inc1;
			}
			else {
				e += inc2;
			}
			x += incx;
			gdiPoint(prc, x, y, rop);
		}
	} else {
		gdiPoint(prc, x, y, rop);
		e = 2 * dx - dy;
		inc1 = 2 * (dx - dy);
		inc2 = 2 * dx;
		for(i = 0; i < dy; i++) {
			if (e >= 0) {
				x += incx;
				e += inc1;
			} else {
				e += inc2;
			}
			y += incy;
			gdiPoint(prc, x, y, rop);
		}
	}
}

//*****************************************************************************
//	Function gdiRectangle(i16 x1, i16 y1, i16 x2, i16 y2, u16 rop)
//
//	Draw rectangle
//
//	parameters:
//		x1			X start position
//		y1			Y start position
//		x2			X end position
//		y2			Y end position
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void gdiRectangle(int x0, int y0, int x1, int y1, uint8_t rop)
{
	gdiLine(0,x0,y0,x1,y0,rop);
	gdiLine(0,x0,y1,x1,y1,rop);
	gdiLine(0,x0,y0,x0,y1,rop);
	gdiLine(0,x1,y0,x1,y1,rop);
}

//*****************************************************************************
//	Function gdiRectangleEx(PGDI_RECT rc, u16 rop)
//
//	Draw rectangle
//
//	parameters:
//		rc			Struct containing the rectangle parameters
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void	gdiRectangleEx(PGDI_RECT rc, uint8_t rop) {

	gdiRectangle(rc->x, rc->y, rc->x + rc->w, rc->y + rc->h,rop);
}

//*****************************************************************************
//	Function gdiCircle(i16 x, i16 y, i16 r, u16 rop)
//
//	Draw circle. This function uses the integer-precision math
//
//	parameters:
//		x			Circle center X position
//		y			Circle center Y position
//		r			Radius
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void	gdiCircle(int x, int y, int r, uint8_t rop) {

int  x1, y1;
uint16_t  a;

	for (a = 0; a < 360; a++) {		
		x1 = r * mthCos(a);
		y1 = r * mthSin(a);
		gdiPoint(0, (x1 / 10000) + x,(y1 / 10000) + y,rop);
	}
}

//*****************************************************************************
//	Function gdiDrawText(PGDI_RECT prc, pu8 ptext, u16 style, u16 rop)
//
//	Draw text inside rectangle
//
//	parameters:
//		prc			Pointer to clipping rectangle
//		ptext		Pointer to text
//		style		Text style (see GDI_WINCAPTION_xx defines)
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void gdiDrawText(PGDI_RECT prc, char *ptext, uint16_t style, uint8_t rop) {

int	l, i, pos, xp;
uint8_t		c;
uint8_t		*ptx;

	l = strLen(ptext) * GDI_SYSFONT_WIDTH;
	switch(style) {
		case GDI_WINCAPTION_RIGHT:		if (l < prc->w) {
											prc->x += (prc->w - l);
										}
										break;
		case GDI_WINCAPTION_CENTER:		if (l < prc->w) {
											prc->x += ((prc->w - l) / 2);
										}
										break;
	}
	l = strLen(ptext);
	xp = 1;//prc->x;
	for (i = 0; i < l; i++) {
		c = *(ptext++);
		if (c >= GDI_SYSFONT_OFFSET) {
			pos = (int) (c - GDI_SYSFONT_OFFSET) * GDI_SYSFONT_BYTEWIDTH * GDI_SYSFONT_HEIGHT;
			ptx = ((unsigned char *)gdiSystemFont) + pos;
			gdiBitBlt(prc, xp, 0, GDI_SYSFONT_WIDTH, GDI_SYSFONT_HEIGHT, ptx, rop);
			xp += GDI_SYSFONT_WIDTH;
			if (xp >= ((prc->x + prc->w) - GDI_SYSFONT_WIDTH)) return;
		}
	}
}

//*****************************************************************************
//	Function gdiDrawTextEx(i16 x, i16 y, pu8 ptext, u16 rop)
//
//	Draw text in X/Y position using system font.
//
//	parameters:
//		x			X start position
//		y			Y start position
//		ptext		Pointer to text
//		rop			Raster operation. See GDI_ROP_xxx defines
//
//	return			none
//*****************************************************************************
void gdiDrawTextEx(int x, int y, char *ptext, uint8_t rop) {

int		l, i, pos, xp;
char		c;
uint8_t		*ptx;

	l = strLen(ptext);
	xp = x;
	for (i = 0; i < l; i++) {
		c = *(ptext++);
		if (c >= GDI_SYSFONT_OFFSET) {
			pos = (int) (c - GDI_SYSFONT_OFFSET) * GDI_SYSFONT_BYTEWIDTH * GDI_SYSFONT_HEIGHT;
			ptx = (uint8_t *)(gdiSystemFont) + pos;
			gdiBitBlt(0, xp, y, GDI_SYSFONT_WIDTH, GDI_SYSFONT_HEIGHT, ptx, rop);
			xp += GDI_SYSFONT_WIDTH;
			if (xp >= VID_PIXELS_X) return;
		}
	}
}
//*****************************************************************************
//	Function gdiDrawWindow(PGDI_WINDOW pwin)
//
//	Draw window
//
//	parameters:
//		pwin		Pointer to windows struct
//
//	return			none
//*****************************************************************************
void	gdiDrawWindow(PGDI_WINDOW pwin) {

int			i;
GDI_RECT	rc, rt;

	gdiCopyRect(&rc,&pwin->rc);
	if (pwin->style & GDI_WINCAPTION) {
		gdiCopyRect(&rt,&pwin->rc);
		rt.h = rt.y + 11;
		rt.x += 2;
		rt.y += 1;
		rc.h += 10;
		for (i = 0; i < 11; i++) {
			gdiLine(0,rc.x, rc.y + i, rc.x + rc.w, rc.y + i, GDI_ROP_COPY);
		}
		if (pwin->style & GDI_WINCLOSEICON) {
			gdiBitBlt(&rc, rc.w - 9, 1, 10, 9, (uint8_t *)gdiCloseBm, GDI_ROP_COPY);
			rt.w -= 11;
		} else {
			rt.w -= 1;
		}
		gdiDrawText(&rt,(char *)pwin->caption, pwin->style & GDI_WINCAPTION_MASK, GDI_ROP_XOR);
	}
	gdiRectangleEx(&rc,GDI_ROP_COPY);
}

