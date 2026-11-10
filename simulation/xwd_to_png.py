#!/usr/bin/env python3
"""Lossless RGB decoder for XWD v7 TrueColor ZPixmap screenshots.

python3 xwd_to_png.py screenshot.xwd screenshot.png [--metadata screenshot.json]
python3 xwd_to_png.py --self-test

Only 24/32 bits-per-pixel with contiguous 8-bit RGB masks are supported.
Image byte order and row padding are honored. No resizing or color adjustment.
Header layout verified against X11/XWDFile.h (100-byte MSB-first header,
12-byte XWDColor records). Non-RGB padding bits are not an alpha channel.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
from pathlib import Path
import struct
import sys

import numpy as np
from PIL import Image

FIELDS = ("header_size file_version pixmap_format pixmap_depth pixmap_width "
          "pixmap_height xoffset byte_order bitmap_unit bitmap_bit_order bitmap_pad "
          "bits_per_pixel bytes_per_line visual_class red_mask green_mask blue_mask "
          "bits_per_rgb colormap_entries ncolors window_width window_height window_x "
          "window_y window_bdrwidth").split()


def decode_xwd(payload: bytes, max_pixels: int = 100_000_000):
    if len(payload) < 100:
        raise ValueError("Truncated XWD header: expected 100 bytes")
    header = dict(zip(FIELDS, struct.unpack(">25I", payload[:100])))
    if header["file_version"] != 7:
        raise ValueError("Expected canonical MSB-first XWD file header, version 7")
    if header["pixmap_format"] != 2 or header["visual_class"] != 4:
        raise ValueError("Only ZPixmap (2), TrueColor (4) is supported; indexed/DirectColor needs another decoder")
    if header["pixmap_depth"] not in (24, 32) or header["bits_per_pixel"] not in (24, 32):
        raise ValueError("Only depth 24/32 and 24/32 bits-per-pixel are supported")
    if header["pixmap_depth"] > header["bits_per_pixel"]:
        raise ValueError("Pixmap depth exceeds bits-per-pixel")
    if header["xoffset"] != 0:
        raise ValueError("Nonzero pixmap xoffset is not supported")
    if header["byte_order"] not in (0, 1) or header["bitmap_bit_order"] not in (0, 1):
        raise ValueError("Invalid byte/bitmap bit order")
    if header["bitmap_pad"] not in (8, 16, 32) or header["bitmap_unit"] not in (8, 16, 32):
        raise ValueError("Invalid bitmap pad/unit")
    if header["bits_per_rgb"] != 8:
        raise ValueError("Only 8-bit RGB components are supported without quantization")
    width, height = header["pixmap_width"], header["pixmap_height"]
    if width < 1 or height < 1 or width*height > max_pixels:
        raise ValueError("Invalid dimensions or maximum pixel count exceeded")
    bpp = header["bits_per_pixel"]; bytes_per_pixel = bpp//8
    stride = header["bytes_per_line"]
    if stride < width*bytes_per_pixel or stride % (header["bitmap_pad"]//8):
        raise ValueError("Invalid row stride / row padding")
    size = header["header_size"]
    if not 101 <= size <= len(payload) or payload[size-1] != 0:
        raise ValueError("Invalid header_size or missing NUL-terminated window name")
    pixel_offset = size + 12*header["ncolors"]
    expected_size = pixel_offset + height*stride
    if len(payload) != expected_size:
        raise ValueError(f"XWD payload size mismatch: expected {expected_size}, got {len(payload)}")
    masks = [header[k] for k in ("red_mask", "green_mask", "blue_mask")]
    shifts = []
    for mask in masks:
        if not mask or mask >= (1 << bpp):
            raise ValueError("Invalid RGB mask")
        shift = (mask & -mask).bit_length()-1
        if mask >> shift != 255:
            raise ValueError("RGB masks must be contiguous and exactly 8 bits wide")
        shifts.append(shift)
    if any(masks[i] & masks[j] for i in range(3) for j in range(i+1, 3)):
        raise ValueError("Overlapping RGB masks")
    rows = np.frombuffer(payload, dtype=np.uint8, offset=pixel_offset).reshape(height, stride)
    packed = rows[:, :width*bytes_per_pixel].reshape(height, width, bytes_per_pixel)
    pixels = np.zeros((height, width), dtype=np.uint32)
    for index in range(bytes_per_pixel):
        bit = 8*index if header["byte_order"] == 0 else 8*(bytes_per_pixel-1-index)
        pixels |= packed[:, :, index].astype(np.uint32) << bit
    rgb = np.stack([((pixels & mask) >> shift).astype(np.uint8)
                    for mask, shift in zip(masks, shifts)], axis=2)
    metadata = dict(format="XWD v7 TrueColor ZPixmap", header=header,
                    window_name=payload[100:size].split(b"\0", 1)[0].decode("utf-8", errors="replace"),
                    image_data_offset=pixel_offset, source_sha256=hashlib.sha256(payload).hexdigest(),
                    rgb_pixel_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
                    output_mode="RGB", pixels_resized=False, color_adjusted=False,
                    unused_pixel_bits="Ignored; XWD TrueColor padding bits are not interpreted as alpha")
    return rgb, metadata


def encode_png(rgb):
    buffer = io.BytesIO()
    Image.fromarray(rgb, mode="RGB").save(buffer, format="PNG")
    encoded = buffer.getvalue()
    with Image.open(io.BytesIO(encoded)) as restored:
        if not np.array_equal(np.asarray(restored), rgb):
            raise AssertionError("PNG round trip changed RGB pixels")
    return encoded


def self_test():
    import unittest

    def fixture(rgb, bpp, order, swap=False, colors=0):
        height, width, _ = rgb.shape
        masks = (0xff, 0xff00, 0xff0000) if swap else (0xff0000, 0xff00, 0xff)
        stride = ((width*(bpp//8)+3)//4)*4
        name = b"synthetic-test\0"
        values = [100+len(name),7,2,24,width,height,0,order,32,order,32,bpp,stride,
                  4,*masks,8,256,colors,width,height,0,0,0]
        rows = bytearray()
        for row in rgb:
            for pixel in row:
                value = sum(int(component) << ((mask & -mask).bit_length()-1)
                            for component, mask in zip(pixel, masks))
                # Padding byte can be nonzero; it must not become transparency.
                if bpp == 32: value |= 0xa7 << 24
                rows.extend(value.to_bytes(bpp//8, "little" if order == 0 else "big"))
            rows.extend(b"\x8f"*(stride-width*(bpp//8)))
        return struct.pack(">25I",*values)+name+b"\0"*(12*colors)+rows

    class XWDTests(unittest.TestCase):
        def test_endianness_padding_masks_and_colors(self):
            expected=np.array([[[255,0,0],[0,255,0],[1,2,3]],[[0,0,255],[255,255,255],[17,128,245]]],np.uint8)
            for bpp in (24,32):
                for order in (0,1):
                    for swap in (False,True):
                        with self.subTest(bpp=bpp,order=order,swap=swap):
                            actual,meta=decode_xwd(fixture(expected,bpp,order,swap,3))
                            np.testing.assert_array_equal(actual,expected)
                            self.assertEqual(meta['header']['ncolors'],3)
                            self.assertTrue(encode_png(actual).startswith(b'\x89PNG'))

        def test_bad_inputs_rejected(self):
            data=fixture(np.zeros((2,3,3),np.uint8),32,0)
            bad_inputs=[data[:40],data[:-1],data+b'\0']
            for index,value in ((1,6),(2,1),(13,5),(14,0x00f000),(6,1)):
                bad=bytearray(data);bad[index*4:(index+1)*4]=struct.pack('>I',value);bad_inputs.append(bytes(bad))
            for bad in bad_inputs:
                with self.assertRaises(ValueError):decode_xwd(bad)
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(XWDTests))
    if not result.wasSuccessful():raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input",nargs="?",help="XWD path, or - for stdin")
    parser.add_argument("output",nargs="?",type=Path)
    parser.add_argument("--metadata",type=Path)
    parser.add_argument("--max-pixels",type=int,default=100_000_000)
    parser.add_argument("--self-test",action="store_true")
    args=parser.parse_args()
    if args.self_test:self_test();return
    if args.input is None or args.output is None:parser.error("input and output are required")
    if args.output.exists() or (args.metadata and args.metadata.exists()):
        raise FileExistsError("Refusing to overwrite output PNG or metadata")
    if args.metadata and args.metadata.resolve()==args.output.resolve():
        raise ValueError("PNG and metadata paths must differ")
    payload=sys.stdin.buffer.read() if args.input=="-" else Path(args.input).read_bytes()
    rgb,metadata=decode_xwd(payload,args.max_pixels);png=encode_png(rgb)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("xb") as target:target.write(png)
    metadata.update(output=str(args.output),png_sha256=hashlib.sha256(png).hexdigest(),
                    png_roundtrip_pixels_verified=True)
    if args.metadata:
        args.metadata.parent.mkdir(parents=True,exist_ok=True)
        with args.metadata.open("x") as target:target.write(json.dumps(metadata,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(metadata,ensure_ascii=False,indent=2))


if __name__=="__main__":main()
