/*************************************************************************/
/*  apng.h                                                               */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef APNG_H
#define APNG_H

#include "core/io/animated_image_loader.h"

class Apng {

	void *source;
	uint32_t (*read_func)(void *source, PoolByteArray &data, int length);
	PoolByteArray data;

	Error get(uint32_t p_length);
	Error get_8(uint8_t *result);
	Error get_16(uint16_t *result);
	Error get_32(uint32_t *result);

	PoolByteArray ihdr_shared; // This part of the IHDR is the same in all the frames.
	PoolByteArray shared_chunks;

	uint32_t width;
	uint32_t height;

	uint8_t *screen;
	uint32_t screen_size;

	int last_undisposed_frame;

	struct Chunk {

		uint32_t size;
		uint32_t type;
	};

	Error _get_chunk(Chunk &p_chunk);
	Error _skip_chunk(const Chunk &p_chunk);

	struct FrameControlChunk {

		uint32_t sequence_number;
		uint32_t width;
		uint32_t height;
		uint32_t x_offset;
		uint32_t y_offset;
		uint16_t delay_num;
		uint16_t delay_den;
		uint8_t dispose_op;
		uint8_t blend_op;
	};

	Error _get_fcTL(FrameControlChunk &fcTL);

	PoolByteArray _get_crc(const PoolByteArray &p_data, int p_begin);
	PoolByteArray _create_frame(const PoolByteArray &p_data, FrameControlChunk &fcTL);
	Error _add_frame(Ref<AnimatedImage> &r_animated_image, PoolByteArray &p_data, FrameControlChunk &fcTL);

	Error _open(void *p_source, AnimatedImage::SourceType source_type);
	Error _load_frames(Ref<AnimatedImage> &r_animated_image, int max_frames = 0);
	Error _close();

	void _clear();

public:
	Error load_from_file_access(Ref<AnimatedImage> &r_animated_image, FileAccess *f, int max_frames = 0);
	Error load_from_buffer(Ref<AnimatedImage> &r_animated_image, const PoolByteArray &p_data, int max_frames = 0);

	Apng();
};

class AnimatedImageLoaderAPNG : public AnimatedImageFormatLoader {

public:
	virtual Error load_animated_image(Ref<AnimatedImage> &r_animated_image, FileAccess *f, int max_frames = 0) const;
	virtual void get_recognized_extensions(List<String> *p_extensions) const;
	virtual bool recognize_format(AnimatedImage::SourceFormat p_format) const;

	AnimatedImageLoaderAPNG();
};

#endif // APNG_H
