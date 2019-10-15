/*************************************************************************/
/*  apng.cpp                                                             */
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

#include "apng.h"

#include <zlib.h>

uint32_t readFromFile(void *source, PoolByteArray &data, int length) {

	FileAccess *f = static_cast<FileAccess *>(source);
	data.resize(length);
	return f->get_buffer(data.write().ptr(), length);
}

struct APNGBuffer { // Used to read the GIF data from a buffer.

	const uint8_t *data;
	int size;
	int index;

	APNGBuffer(const PoolByteArray &p_data) {
		data = p_data.read().ptr();
		size = p_data.size();
		index = 0;
	}

	APNGBuffer(const uint8_t *p_data, int p_size) {
		data = p_data;
		size = p_size;
		index = 0;
	}
};

uint32_t readFromBuffer(void *source, PoolByteArray &data, int length) {

	APNGBuffer *f = static_cast<APNGBuffer *>(source);
	if (f->index + length > f->size)
		length = f->size - f->index;

	data.resize(length);
	memcpy(data.write().ptr(), &f->data[f->index], length);

	f->index += length;
	return length;
}

Error Apng::get(uint32_t p_length) {

	uint32_t read_length = read_func(source, data, p_length);
	if (read_length != p_length) {

		ERR_PRINT("Unexpected end of file.")
		return ERR_FILE_CORRUPT;
	}

	return OK;
}

Error Apng::get_8(uint8_t *result) {

	if (get(1) != OK)
		return ERR_FILE_CORRUPT;

	*result = data[0];
	return OK;
}

Error Apng::get_16(uint16_t *result) {

	if (get(2) != OK)
		return ERR_FILE_CORRUPT;

	*result = uint16_t(uint8_t(data[0]) << 8 | uint8_t(data[1]));
	return OK;
}

Error Apng::get_32(uint32_t *result) {

	if (get(4) != OK)
		return ERR_FILE_CORRUPT;

	*result = uint32_t(uint8_t(data[0]) << 24 | uint8_t(data[1]) << 16 | uint8_t(data[2]) << 8 | uint8_t(data[3]));
	return OK;
}

#define GET(length)            \
	{                          \
		if (get(length) != OK) \
			return FAILED;     \
	}

#define GET_8(result)             \
	{                             \
		if (get_8(&result) != OK) \
			return FAILED;        \
	}

#define GET_16(result)             \
	{                              \
		if (get_16(&result) != OK) \
			return FAILED;         \
	}

#define GET_32(result)             \
	{                              \
		if (get_32(&result) != OK) \
			return FAILED;         \
	}

Error Apng::_get_chunk(Chunk &p_chunk) {

	GET(8); // Use a single read.
	p_chunk.size = uint32_t(uint8_t(data[0]) << 24 | uint8_t(data[1]) << 16 | uint8_t(data[2]) << 8 | uint8_t(data[3]));
	p_chunk.type = uint32_t(uint8_t(data[4]) << 24 | uint8_t(data[5]) << 16 | uint8_t(data[6]) << 8 | uint8_t(data[7]));
	return OK;
}

#define CHECKSUM 4

Error Apng::_skip_chunk(const Chunk &p_chunk) {

	GET(p_chunk.size + CHECKSUM); // All the chunks end with a 4 bytes checksum.
	return OK;
}

Error Apng::_get_fcTL(FrameControlChunk &fcTL) { // The file pointer must be at the beginning of the fcTL data block.

	GET_32(fcTL.sequence_number);
	GET_32(fcTL.width);
	GET_32(fcTL.height);
	GET_32(fcTL.x_offset);
	GET_32(fcTL.y_offset);
	GET_16(fcTL.delay_num);
	GET_16(fcTL.delay_den);
	GET_8(fcTL.dispose_op);
	GET_8(fcTL.blend_op);
	GET(CHECKSUM); // Skip the checksum.
	return OK;
}

// Calculate the checksum of all the data starting from position p_begin.
PoolByteArray Apng::_get_crc(const PoolByteArray &p_data, int p_begin) {

	ERR_FAIL_INDEX_V(p_begin, p_data.size(), PoolByteArray());

	uLong crc = crc32(0L, Z_NULL, 0);
	crc = crc32(crc, &p_data.read().ptr()[p_begin], p_data.size() - p_begin);

	PoolByteArray result;
	result.push_back(uint8_t(crc >> 24));
	result.push_back(uint8_t(crc >> 16));
	result.push_back(uint8_t(crc >> 8));
	result.push_back(uint8_t(crc));
	return result;
}

#define COMMON_HEADER_SIZE 16
#define IHDR_START 12
#define IEND_SIZE 12

// Append an integer to the result.
#define WRITE_32(val)                         \
	{                                         \
		result.push_back(uint8_t(val >> 24)); \
		result.push_back(uint8_t(val >> 16)); \
		result.push_back(uint8_t(val >> 8));  \
		result.push_back(uint8_t(val));       \
	}

// Create a valid individual PNG image from the data of a single frame.
PoolByteArray Apng::_create_frame(const PoolByteArray &p_data, FrameControlChunk &fcTL) {

	PoolByteArray result;

	// All the APNG file begin with this header.
	uint8_t header[COMMON_HEADER_SIZE] = { 137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 73, 72, 68, 82 };
	result.resize(COMMON_HEADER_SIZE);
	for (int i = 0; i < COMMON_HEADER_SIZE; i++)
		result.set(i, header[i]);

	WRITE_32(fcTL.width); // IHDR width and height.
	WRITE_32(fcTL.height);

	result.append_array(ihdr_shared);
	result.append_array(_get_crc(result, IHDR_START));

	result.append_array(shared_chunks); // All the shared chunks after the IHDR and before the IDAT.

	WRITE_32(p_data.size());

	uint32_t idat_index = result.size();

	uint8_t idat[4] = { 73, 68, 65, 84 }; // IDAT
	for (int i = 0; i < 4; i++)
		result.push_back(idat[i]);

	result.append_array(p_data);

	result.append_array(_get_crc(result, idat_index));

	uint8_t iend[IEND_SIZE] = { 0, 0, 0, 0, 73, 69, 78, 68, 174, 66, 96, 130 };
	for (int i = 0; i < IEND_SIZE; i++)
		result.push_back(iend[i]);

	return result;
}

#define RGBA 4

#define APNG_BLEND_OP_SOURCE 0
#define APNG_BLEND_OP_OVER 1

#define APNG_DISPOSE_OP_NONE 0
#define APNG_DISPOSE_OP_BACKGROUND 1
#define APNG_DISPOSE_OP_PREVIOUS 2

Error Apng::_add_frame(Ref<AnimatedImage> &r_animated_image, PoolByteArray &p_data, FrameControlChunk &fcTL) {

	if (fcTL.delay_den == 0) // The default value for a denominator of 0 is 100.
		fcTL.delay_den = 100;
	float delay = float(fcTL.delay_num) / float(fcTL.delay_den);
	if (delay == 0)
		delay = 0.05; // Default delay;

	PoolByteArray frame_data = _create_frame(p_data, fcTL);

	Ref<Image> frame = Image::_png_mem_loader_func(frame_data.read().ptr(), frame_data.size());
	if (frame.is_null())
		return ERR_FILE_CORRUPT;

	frame->convert(Image::FORMAT_RGBA8);

	int row_size = fcTL.width * RGBA;
	const uint8_t *r = frame->get_data().read().ptr();

	// Each frame has a different size and offset.
	if (fcTL.blend_op == APNG_BLEND_OP_SOURCE) { // Blit.

		for (uint32_t y = 0; y < fcTL.height; y++) {

			int read_index = (y * fcTL.width) * RGBA;
			int write_y = y + fcTL.y_offset;
			int write_index = (write_y * width + fcTL.x_offset) * RGBA;
			memcpy(&screen[write_index], &r[read_index], row_size);
		}
	} else { // Blend.

		for (uint32_t y = 0; y < fcTL.height; y++) {

			for (uint32_t x = 0; x < fcTL.width; x++) {

				int read_index = (y * fcTL.width + x) * RGBA;
				int write_y = y + fcTL.y_offset;
				int write_x = x + fcTL.x_offset;
				int write_index = (write_y * width + write_x) * RGBA;

				double fa = r[read_index + 3] / 255.0; // Frame alpha.
				double sa = screen[write_index + 3] / 255.0; // Screen alpha.
				double isa = sa * (1.0 - fa);

				for (uint32_t c = 0; c < 3; c++) // RGB.
					screen[write_index + c] = fa * r[read_index + c] + isa * screen[write_index + c];

				screen[write_index + 3] = (fa + isa) * 255.0; // Alpha.
			}
		}
	}

	PoolByteArray screen_data;
	screen_data.resize(screen_size);
	memcpy(screen_data.write().ptr(), screen, screen_size);

	frame->create(width, height, false, Image::FORMAT_RGBA8, screen_data);

	r_animated_image->add_frame(frame, delay);

	switch (fcTL.dispose_op) {

		case APNG_DISPOSE_OP_BACKGROUND: { // Clear the area of the current frame.

			for (uint32_t y = 0; y < fcTL.height; y++) {

				int write_y = y + fcTL.y_offset;
				int write_index = (write_y * width + fcTL.x_offset) * RGBA;
				memset(&screen[write_index], 0, row_size);
			}
		} break;
		case APNG_DISPOSE_OP_PREVIOUS: { // Reset the area of the current frame to the last undisposed frame.

			if (last_undisposed_frame == -1) { // This is the first frame, clear the area.

				for (uint32_t y = 0; y < fcTL.height; y++) {

					int write_y = y + fcTL.y_offset;
					int write_index = (write_y * width + fcTL.x_offset) * RGBA;
					memset(&screen[write_index], 0, row_size);
				}
			} else {

				PoolByteArray previous_data = r_animated_image->get_image(last_undisposed_frame)->get_data();
				const uint8_t *pr = previous_data.read().ptr();

				for (uint32_t y = 0; y < fcTL.height; y++) {

					int write_y = y + fcTL.y_offset;
					int write_index = (write_y * width + fcTL.x_offset) * RGBA;
					memcpy(&screen[write_index], &pr[write_index], row_size);
				}
			}
		} break;
		default: { // Do nothing.

			last_undisposed_frame = r_animated_image->get_frames() - 1;
		}
	}

	return OK;
}

#define HEADER_SIZE 8
#define IHDR_SIZE 13
#define IHDR_SHARED_SIZE 5

#define IHDR 1229472850
#define ACTL 1633899596 // acTL
#define FCTL 1717785676 // fcTL
#define IDAT 1229209940
#define FDAT 1717846356 // fdAT
#define IEND 1229278788

Error Apng::_open(void *p_source, AnimatedImage::SourceType source_type) {

	source = p_source;
	read_func = source_type == AnimatedImage::FILE ? readFromFile : readFromBuffer;

	GET(HEADER_SIZE);

	uint8_t apng_header[HEADER_SIZE] = { 137, 80, 78, 71, 13, 10, 26, 10 };
	if (memcmp(data.read().ptr(), &apng_header[0], HEADER_SIZE) != 0) {

		ERR_PRINT("Unrecognized apng header.");
		return ERR_FILE_UNRECOGNIZED;
	}

	Chunk chunk;
	if (_get_chunk(chunk) != OK)
		return FAILED;

	if (chunk.type != IHDR || chunk.size != IHDR_SIZE) {

		ERR_PRINT("Wrong IHDR data.");
		return ERR_FILE_UNRECOGNIZED;
	}

	GET_32(width);
	GET_32(height);

	if (width > Image::MAX_WIDTH || height > Image::MAX_HEIGHT)
		return FAILED;

	GET(IHDR_SHARED_SIZE); // All the frames share the same IHDR expect the widht and height parameters.
	ihdr_shared.resize(IHDR_SHARED_SIZE);
	for (int i = 0; i < IHDR_SHARED_SIZE; i++)
		ihdr_shared.set(i, data[i]);

	GET(CHECKSUM); // Skip the checksum of the IHDR.

	bool fctl_found = false;
	bool actl_found = false;

	do { // Search for frames till the first fcTL.

		if (_get_chunk(chunk) != OK)
			return FAILED;

		switch (chunk.type) {
			case FCTL: {

				fctl_found = true;
				continue;
			} break;
			case ACTL: {

				actl_found = true;
			} break;
			case IDAT: { // An IDAT before an FCTL is not part of the animation.
			} break;
			default: {

				shared_chunks.append_array(data);
			}
		}

		if (_skip_chunk(chunk) != OK) // Skip to the next chunk.
			return FAILED;

		switch (chunk.type) {
			case ACTL:
			case IDAT: {
			} break;
			default: {

				shared_chunks.append_array(data);
			}
		}

	} while (!fctl_found);

	if (!actl_found) // The file isn't an animated image.
		return FAILED;

	return OK;
}

Error Apng::_load_frames(Ref<AnimatedImage> &r_animated_image, int max_frames) {

	screen_size = width * height * RGBA;
	screen = memnew_arr(uint8_t, screen_size);
	memset(screen, 0, screen_size); // Clear the screen.

	// The pointer should be after the header of the first fcTL.
	Chunk chunk;
	FrameControlChunk fcTL;
	if (_get_fcTL(fcTL) != OK)
		return FAILED;

	uint32_t current_frame = 0;
	if (fcTL.sequence_number != current_frame) // APNG with out of order frames.
		return ERR_FILE_CORRUPT;
	current_frame++;

	PoolByteArray frame_data;

	bool loaded = false;
	do {

		if (_get_chunk(chunk) != OK)
			return FAILED;

		switch (chunk.type) {
			case FCTL: {

				if (_add_frame(r_animated_image, frame_data, fcTL) != OK)
					return FAILED;

				if (r_animated_image->get_frames() == max_frames) {

					loaded = true;
					continue;
				}

				frame_data.resize(0);

				if (_get_fcTL(fcTL) != OK)
					return FAILED;

				if (fcTL.sequence_number != current_frame)
					return ERR_FILE_CORRUPT;

				if (fcTL.x_offset + fcTL.width > width)
					return ERR_FILE_CORRUPT;

				if (fcTL.y_offset + fcTL.height > height)
					return ERR_FILE_CORRUPT;

				current_frame++;
			} break;
			case IDAT: { // The frame data can be splitted in multiple consecutive IDAT or FDAT.

				GET(chunk.size);
				frame_data.append_array(data);
				GET(CHECKSUM);
			} break;
			case FDAT: {

				uint32_t sequence_number; // fdAT chunks store the sequence_number in the first 4 bytes.
				GET_32(sequence_number);

				if (sequence_number != current_frame)
					return ERR_FILE_CORRUPT;

				chunk.size -= 4; // Remove the 4 bytes of sequence number from the chunck size.

				GET(chunk.size);
				frame_data.append_array(data);
				GET(CHECKSUM);
				current_frame++;
			}; break;
			case IEND: {

				if (_add_frame(r_animated_image, frame_data, fcTL) != OK)
					return FAILED;

				loaded = true;
			}; break;
			default: {

				if (_skip_chunk(chunk) != OK) // Skip to the next chunk.
					return FAILED;
			}
		}
	} while (!loaded);

	return OK;
}

Error Apng::_close() {

	_clear();
	return OK;
}

void Apng::_clear() {

	if (screen != NULL) {

		memdelete_arr(screen);
		screen = NULL;
		screen_size = 0;
	}

	read_func = NULL;
	width = 0;
	height = 0;
	last_undisposed_frame = -1;
	data.resize(0);
	ihdr_shared.resize(0);
	shared_chunks.resize(0);
}

Error Apng::load_from_file_access(Ref<AnimatedImage> &r_animated_image, FileAccess *f, int max_frames) {

	Error err;

	err = _open(f, AnimatedImage::FILE);
	if (err != OK)
		return ERR_FILE_CORRUPT;

	err = _load_frames(r_animated_image, max_frames);
	if (err != OK) {

		_close();
		return ERR_FILE_CORRUPT;
	}

	err = _close();
	if (err != OK)
		return ERR_FILE_CORRUPT;

	return OK;
}

Error Apng::load_from_buffer(Ref<AnimatedImage> &r_animated_image, const PoolByteArray &p_data, int max_frames) {

	APNGBuffer f = APNGBuffer(p_data);

	Error err;
	err = _open(&f, AnimatedImage::BUFFER);
	if (err != OK)
		return ERR_FILE_CORRUPT;

	err = _load_frames(r_animated_image, max_frames);
	if (err != OK) {

		_close();
		return ERR_FILE_CORRUPT;
	}

	err = _close();
	if (err != OK)
		return ERR_FILE_CORRUPT;

	return OK;
}

Apng::Apng() {

	screen = NULL;
	screen_size = 0;
	_clear();
}

///////////////

static Error _load_apng(Ref<AnimatedImage> &r_animated_image, const Variant &source, int max_frames) {

	Apng apng;

	if (source.get_type() == Variant::STRING) {
		Error err;
		FileAccess *f = FileAccess::open(source, FileAccess::READ, &err);
		if (!f) {

			ERR_PRINTS("Error opening file '" + String(source) + "'.");
			return err;
		}

		err = apng.load_from_file_access(r_animated_image, f, max_frames);

		f->close();
		memdelete(f);
		return err;
	} else {

		return apng.load_from_buffer(r_animated_image, source, max_frames);
	}
}

///////////////

Error AnimatedImageLoaderAPNG::load_animated_image(Ref<AnimatedImage> &r_animated_image, FileAccess *f, int max_frames) const {

	Apng apng;
	return apng.load_from_file_access(r_animated_image, f, max_frames);
}

void AnimatedImageLoaderAPNG::get_recognized_extensions(List<String> *p_extensions) const {

	p_extensions->push_back("apng");
}

bool AnimatedImageLoaderAPNG::recognize_format(AnimatedImage::SourceFormat p_format) const {

	return p_format == AnimatedImage::APNG;
}

AnimatedImageLoaderAPNG::AnimatedImageLoaderAPNG() {

	AnimatedImage::_load_apng = _load_apng;
}
