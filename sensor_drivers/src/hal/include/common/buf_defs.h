/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 VeriSilicon Holdings Co., Ltd. ("VeriSilicon")
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/
#ifndef __CAMERA_DEVICE_BUF_DEFS_H__
#define __CAMERA_DEVICE_BUF_DEFS_H__

#include <ebase/types.h>

#define UNIQUE_ENUM_NAME(u)			assert_static__##u
#define GET_ENUM_NAME(x)			UNIQUE_ENUM_NAME(x)
#define DCT_ASSERT_STATIC(e)			enum { GET_ENUM_NAME(__LINE__) = 1/(e) }
#define BUFF_POOL_MAX_INPUT_BUF_NUMBER		(8)
#define BUFF_POOL_MAX_OUTPUT_BUF_NUMBER		(16)
#define PIC_BUFFER_ALIGN			(0x400)
#define PIC_WIDTH_ALIGN				(16)
#define ALIGN_16BYTES(width)			(((width) + (0xFU)) & (~(0xFU)))
#define PIC_BUFFER_MEM_SIZE_MAX			(512 * 1024 * 1024)
#define PIC_BUFFER_NUM_INPUT			(1)
#define PIC_BUFFER_SIZE_INPUT			(110 * 1024 * 1024)

#ifdef SW_TEST
#define PIC_BUFFER_NUM_LARGE_IMAGE		(4)
#else
#define PIC_BUFFER_NUM_LARGE_IMAGE		(7)
#endif

#define PIC_BUFFER_SIZE_LARGE_IMAGE		(42 * 1024 * 1024)
#define PIC_BUFFER_NUM_SMALL_IMAGE		(13)
#define PIC_BUFFER_SIZE_SMALL_IMAGE		(8 * 1024 * 1024)

DCT_ASSERT_STATIC(((PIC_BUFFER_NUM_INPUT * PIC_BUFFER_SIZE_INPUT) + (PIC_BUFFER_NUM_LARGE_IMAGE *
	PIC_BUFFER_SIZE_LARGE_IMAGE) + (PIC_BUFFER_NUM_SMALL_IMAGE * PIC_BUFFER_SIZE_SMALL_IMAGE))
	< PIC_BUFFER_MEM_SIZE_MAX);

#ifdef ISP_SINGLE_DOM_OVER_MP
#define PIC_BUFFER_NUM_MAIN_SENSOR	(10)
#define PIC_BUFFER_SIZE_MAIN_SENSOR	(16 * 1024 * 1024)
#define PIC_BUFFER_NUM_SELF1_SENSOR	(0)
#define PIC_BUFFER_SIZE_SELF1_SENSOR	(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_SELF2_SENSOR	(0)
#define PIC_BUFFER_SIZE_SELF2_SENSOR	(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_MCMWR_SENSOR	(6)
#define PIC_BUFFER_SIZE_MCMWR_SENSOR	(8 * 1024 * 1024)
#define PIC_BUFFER_NUM_METADATA		(10)
#define PIC_BUFFER_SIZE_METADATA	(8 * 1024 * 1024)

#ifdef ISP_MI_HDR
#define PIC_BUFFER_NUM_HDR_SENSOR	(6)
#define PIC_BUFFER_SIZE_HDR_SENSOR	(8 * 1024 * 1024)
#endif

#define PIC_BUFFER_NUM_PP_PATH_RAW_SENSOR	(0)
#define PIC_BUFFER_SIZE_PP_PATH_RAW_SENSOR	(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_MAIN_SENSOR_IMGSTAB	(10)
#define PIC_BUFFER_SIZE_MAIN_SENSOR_IMGSTAB	(6 * 1024 * 1024)
#define PIC_BUFFER_SIZE_MAIN_SENSOR_IMGSTAB	(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_SELF_SENSOR_IMGSTAB	(10)
#define PIC_BUFFER_SIZE_SELF_SENSOR_IMGSTAB	(6 * 1024 * 1024)

DCT_ASSERT_STATIC(((PIC_BUFFER_NUM_MAIN_SENSOR*PIC_BUFFER_SIZE_MAIN_SENSOR) +
	(PIC_BUFFER_NUM_SELF1_SENSOR*PIC_BUFFER_SIZE_SELF1_SENSOR)) < PIC_BUFFER_MEM_SIZE_MAX);
#else
#define PIC_BUFFER_NUM_MAIN_SENSOR		(6)
#define PIC_BUFFER_SIZE_MAIN_SENSOR		(64 * 1024 * 1024)
#define PIC_BUFFER_NUM_SELF1_SENSOR		(8)
#define PIC_BUFFER_SIZE_SELF1_SENSOR		(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_SELF2_SENSOR		(6)
#define PIC_BUFFER_SIZE_SELF2_SENSOR		(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_MCMWR_SENSOR		(6)
#define PIC_BUFFER_SIZE_MCMWR_SENSOR		(8 * 1024 * 1024)

#ifdef ISP_MI_HDR
#define PIC_BUFFER_NUM_HDR_SENSOR	(6)
#define PIC_BUFFER_SIZE_HDR_SENSOR	(8 * 1024 * 1024)
#endif

#define PIC_BUFFER_NUM_METADATA			(10)
#define PIC_BUFFER_SIZE_METADATA		(8 * 1024 * 1024)
#define PIC_BUFFER_NUM_PP_PATH_RAW_SENSOR	(2)
#define PIC_BUFFER_SIZE_PP_PATH_RAW_SENSOR	(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_MAIN_SENSOR_IMGSTAB	(10)
#define PIC_BUFFER_SIZE_MAIN_SENSOR_IMGSTAB	(6 * 1024 * 1024)
#define PIC_BUFFER_NUM_SELF_SENSOR_IMGSTAB	(10)
#define PIC_BUFFER_SIZE_SELF_SENSOR_IMGSTAB	(6 * 1024 * 1024)

DCT_ASSERT_STATIC(((PIC_BUFFER_NUM_MAIN_SENSOR*PIC_BUFFER_SIZE_MAIN_SENSOR) +
	(PIC_BUFFER_NUM_SELF1_SENSOR*PIC_BUFFER_SIZE_SELF1_SENSOR)) < PIC_BUFFER_MEM_SIZE_MAX);
#endif

#define MEDIA_BUF_ALIGN(buf, align)	(((buf)+((align)-1U)) & ~((align)-1U))
#define PIC_EXP_NUM_MAX			(4)
#define METADATA_MAX_NUM		(3)

typedef void *HalHandle_t;
typedef void *BufMgmtHandle_t;

/*****************************************************************************
 * @brief   Enumeration type to specify the swap control.
 *
 * @note    This enumeration type is used to specify the swap control.
 *
 *****************************************************************************/
typedef enum {
	BUFF_MODE_INVALID		= 0,
	BUFF_MODE_USRPTR		= 1,
	BUFF_MODE_RESMEM		= 2,
	BUFF_MODE_MAX,
	DUMMY_BUFF_MODE			= 0xdeadfeed
} BUFF_MODE;

typedef enum {
	PIC_BUF_MI_SWAP_INVALID		= -1,
	PIC_BUF_MI_NO_SWAP		= 0,
	PIC_BUF_MI_SWAP_BYTES		= 1,
	PIC_BUF_MI_SWAP_WORDS		= 2,
	PIC_BUF_MI_SWAP_DOUBLE_WORDS	= 4,
	PIC_BUF_MI_SWAP_FOUR_WORDS	= 8,
	PIC_BUF_MI_SWAP_MAX,
	DUMMY_PIC_BUF_MI_SWAP		= 0xdeadfeed
} PicBufMiDataSwap_t;

/*****************************************************************************
 *          PicBufType_t
 *
 * @brief   The type of image data a picture buffer holds.
 *
 * @note    MVDU_FXQuad requires PIC_BUF_TYPE_YCbCr422 in PIC_BUF_LAYOUT_SEMIPLANAR mode.
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_TYPE_INVALID		= 0,
	PIC_BUF_TYPE_JPEG		= 2,
	PIC_BUF_TYPE_YCbCr444		= 3,
	PIC_BUF_TYPE_YCbCr422		= 4,
	PIC_BUF_TYPE_YCbCr420		= 5,
	PIC_BUF_TYPE_YCbCr400		= 6,
	PIC_BUF_TYPE_RGB888		= 7,
	PIC_BUF_TYPE_RGB666		= 8,
	PIC_BUF_TYPE_RGB565		= 9,
	PIC_BUF_TYPE_RAW8		= 10,
	PIC_BUF_TYPE_RAW12		= 11,
	PIC_BUF_TYPE_RAW10		= 13,
	PIC_BUF_TYPE_RAW14		= 14,
	PIC_BUF_TYPE_RAW16		= 15,
	PIC_BUF_TYPE_RAW20		= 16,
	PIC_BUF_TYPE_RAW20_COMPRESS	= 17,
	PIC_BUF_TYPE_RAW24		= 18,
	PIC_BUF_TYPE_RAW24_COMPRESS	= 19,
	PIC_BUF_TYPE_META		= 20,
	PIC_BUF_TYPE_DATA		= 21,
	PIC_BUF_TYPE_YCbCr32		= 22,
	PIC_BUF_TYPE_RGB32		= 23,
	PIC_BUF_TYPE_DPCC32		= 24,
	DUMMY_PIC_BUF_TYPE		= 0xdeadfeed
} PicBufType_t;

/*****************************************************************************
 * @brief   RDCE bit depth enum.
 *
 * @note    This is a enum of RDCE bit depth.
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_RDCE_BIT_DEPTH_INVALID	= -1,
	PIC_BUF_RDCE_BIT_DEPTH_RAW8,
	PIC_BUF_RDCE_BIT_DEPTH_RAW10,
	PIC_BUF_RDCE_BIT_DEPTH_RAW12,
	PIC_BUF_RDCE_BIT_DEPTH_RAW14,
	PIC_BUF_RDCE_BIT_DEPTH_RAW16,
	PIC_BUF_RDCE_BIT_DEPTH_MAX,
	DUMMY_PIC_BUF_RDCE_BIT_DEPTH	= 0xdeadfeed
} PicBufRdceType_t;

/*****************************************************************************
 * @brief   RDCE bayer pattern enum.
 *
 * @note    This is a enum of RDCE bayer pattern.
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_RDCE_BPT_INVALID	= -1,
	PIC_BUF_RDCE_BPT_RGGB,
	PIC_BUF_RDCE_BPT_GRBG,
	PIC_BUF_RDCE_BPT_GBRG,
	PIC_BUF_RDCE_BPT_BGGR,
	PIC_BUF_RDCE_BPT_MAX,
	DUMMY_PIC_BUF_RDCE_BPT		= 0xdeadfeed
} PicBufRdceBayerPat_t;

/*****************************************************************************
 *          PicBufLayout_t
 *
 * @brief   The layout of the image data a picture buffer holds.
 *
 * @note    MVDU_FXQuad requires PIC_BUF_TYPE_YCbCr422 in PIC_BUF_LAYOUT_SEMIPLANAR mode.
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_LAYOUT_INVALID		= 0,
	PIC_BUF_LAYOUT_COMBINED		= 0x10,
	PIC_BUF_LAYOUT_BAYER_RGRGGBGB	= 0x11,
	PIC_BUF_LAYOUT_BAYER_GRGRBGBG	= 0x12,
	PIC_BUF_LAYOUT_BAYER_GBGBRGRG	= 0x13,
	PIC_BUF_LAYOUT_BAYER_BGBGGRGR	= 0x14,
	PIC_BUF_LAYOUT_SEMIPLANAR	= 0x20,
	PIC_BUF_LAYOUT_PLANAR		= 0x30,
	PIC_BUF_LAYOUT_META_DATA	= 0x40,
	DUMMY_PIC_BUF_LAYOUTT		= 0xdeadfeed
} PicBufLayout_t;

/*****************************************************************************
 *          PixelDataAlignMode_t
 *
 * @brief   The align mode of the image data a picture buffer holds.
 *
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_DATA_ALIGN_MODE_INVALID		= -1,
	PIC_BUF_DATA_UNALIGN_MODE		= 0,
	PIC_BUF_DATA_ALIGN_128BIT_MODE		= 1,
	PIC_BUF_DATA_ALIGN_DOUBLE_WORD		= 1,
	PIC_BUF_DATA_ALIGN_WORD			= 2,
	PIC_BUF_DATA_ALIGN_16BIT_MODE		= 2,
	PIC_BUF_DATA_ALIGN_MODE_MAX,
	DUMMY_PIC_BUF_DATA_ALIGN		= 0xdeadfeed
} PicBufAlign_t;

typedef enum {
	PIC_BUF_DATA_YUV_BIT_MAX_INVALID	= -1,
	PIC_BUF_DATA_YUV_8_BIT			= 0,
	PIC_BUF_DATA_YUV_10_BIT			= 1,
	PIC_BUF_DATA_YUV_12_BIT			= 2,
	PIC_BUF_DATA_YUV_BIT_MAX,
	DUMMY_PIC_BUF_DATA_YUV			= 0xdeadfeed
} PicBufYUVBIT_t;

/*****************************************************************************
 * @brief   Enumeration type to specify the order of YUV or RGB channel,
 *          Right now only surpport RGB888 format
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_CHANNEL_ORDER_INVALID		= -1,
	PIC_BUF_CHANNEL_ORDER_YUV		= 0,
	PIC_BUF_CHANNEL_ORDER_YVU		= 1,
	PIC_BUF_CHANNEL_ORDER_UYV		= 2,
	PIC_BUF_CHANNEL_ORDER_VYU		= 3,
	PIC_BUF_CHANNEL_ORDER_UVY		= 4,
	PIC_BUF_CHANNEL_ORDER_VUY		= 5,
	PIC_BUF_CHANNEL_ORDER_MAX		= 6,
	DUMMY_PIC_BUF_CHANNEL_ORDER		= 0xdeadfeed
} PicBufYuvOrder_t;

/*****************************************************************************
 * @brief   Picture buffer compress lossless/lossy  mode
 *
 *****************************************************************************/
typedef enum {
	PIC_BUF_LOSS_MODE_INVALID		= -1,
	PIC_BUF_LOSS_LESS_MODE,
	PIC_BUF_LOSSY_MODE,
	PIC_BUF_LOSS_MODE_MAX,
	DUMMY_PIC_BUF_LOSS_MODE			= 0xdeadfeed
} PicBufLossMode_t;

/*****************************************************************************
 * @brief   Union type to specify the raw and yuv swap control.
 *
 * @note    This union type is used to specify the raw and yuv swap control.
 *
 *****************************************************************************/
typedef union {
	struct {
		PicBufMiDataSwap_t	y;
		PicBufMiDataSwap_t	u;
		PicBufMiDataSwap_t	v;
	} yuvSwap;
	PicBufMiDataSwap_t rawSwap;
} PicBufMiSwap_t;

typedef struct {
	uint32_t	buff_size;
	int		width;
	int		height;
	int		format;
	int		widthBytes;
	int		cbCrWidthBytes;
	int		cbCrHeightPixel;
	int		cbWidthBytes;
	int		cbHeightPixel;
	int		crWidthBytes;
	int		crHeightPixel;
	int		yuvOrder;
	int		alignMode;
	uint8_t		bitWidth;
	uint8_t		*bufferInstance;
	uint32_t	buff_address;
	PicBufMiSwap_t swap;
} BufIdentity;

typedef struct {
	void		*p_next;
	void		*p_address;
	uint32_t	base_address;
	BufIdentity	buf_id;
	uint32_t	size;
	uint32_t	buffer_flags;
	int64_t		time_stamp;
	void		*p_meta_data;
} ScmiBuffer;

typedef struct {
	uint32_t	baseAddress;
	uint32_t	baseSize;
	uint32_t	lockCount;
	void		*pOwner;
	bool_t		isFull;
	void		*pMetaData;
	uint8_t		index;
	BUFF_MODE	bufMode;
	void		*pIplAddress;
#ifdef WITH_FLEXA
	int		fd;
	void		*mobj;
#endif
	ScmiBuffer	buf;
} MediaBuffer_t;

typedef struct {
	uint8_t		*pData;
	uint32_t	BaseAddress;
	uint32_t	PicWidthPixel;
	uint32_t	PicWidthBytes;
	uint32_t	PicHeightPixel;
	uint8_t		PixelDataAlignMode;
	uint8_t		bitWidth;
} PicBufPlane_t;

typedef struct {
	uint64_t	frameCount;
	uint64_t	timestamp_sof;
	uint64_t	timestamp_eof;
	float		sensorGain[PIC_EXP_NUM_MAX];
	float		expoInfo[PIC_EXP_NUM_MAX];
} PicBufMetadataInfo_t;

typedef struct {
	uint8_t		winNum;
	uint8_t		*pBuffer[METADATA_MAX_NUM];
	uint32_t	address[METADATA_MAX_NUM];
	uint32_t	bufferSize[METADATA_MAX_NUM];
} MetadataBufInfo_t;

typedef struct {
	bool_t			compressed;
	PicBufLossMode_t	lossMode;
	PicBufRdceType_t	bitdepth;
	short			targetSize;
	short			bitThresh;
	PicBufRdceBayerPat_t	bayerPattern;
	uint32_t		crcValue;
} PicBufCmpInfo_t;

typedef struct PicBufMetaData_s {
	PicBufType_t				Type;
	PicBufLayout_t				Layout;
	uint32_t				Align;
	int64_t					TimeStampUs;
	struct PicBufMetaData_s			*pNext3D;
	PicBufCmpInfo_t				compressInfo;
	PicBufMetadataInfo_t			metaInfo;
	PicBufYuvOrder_t			yuvOrder;
	PicBufMiSwap_t				swap;
	union Data_u {
		struct data_s {
			uint8_t			*pData;
			uint32_t		BaseAddress;
			uint32_t		DataSize;
		} data;

		struct data_meta {
			uint8_t			*pData;
			uint32_t		BaseAddress;
			uint32_t		DataSize;
			PicBufPlane_t		plane[METADATA_MAX_NUM];
			MetadataBufInfo_t	metaBufInfo;
		} meta;

		PicBufPlane_t raw;

		struct jpeg_s {
			uint8_t			*pHeader;
			uint32_t		HeaderSize;
			uint8_t			*pData;
			uint32_t		BaseAddress;
			uint32_t		DataSize;
		} jpeg;

		union YCbCr_u {
			PicBufPlane_t combined;
			struct semiplanar_s {
				PicBufPlane_t	Y;
				PicBufPlane_t	CbCr;
				uint8_t		bitWidth;
			} semiplanar;
			struct planar_YUV_s {
				PicBufPlane_t	Y;
				PicBufPlane_t	Cb;
				PicBufPlane_t	Cr;
			} planar;
		} YCbCr;

		union RGB_u {
			PicBufPlane_t combined;
			struct planar_RGB_s {
				PicBufPlane_t	R;
				PicBufPlane_t	G;
				PicBufPlane_t	B;
			} planar;
		} RGB;
#ifdef ISP_MI_BP
		union BAYER_u {
			struct planar_BAYER_s {
				PicBufPlane_t	R;
				PicBufPlane_t	Gr;
				PicBufPlane_t	Gb;
				PicBufPlane_t	B;
			} planar;
		} BAYER;
#endif
	} Data;
} PicBufMetaData_t;

#endif
