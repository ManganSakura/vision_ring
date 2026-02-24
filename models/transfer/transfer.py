#!/usr/bin/env python3
from PIL import Image

def make_transparent(img_path, output_path):
    img = Image.open(img_path).convert("RGBA")
    datas = img.getdata()

    new_data = []
    for item in datas:
        # 如果像素接近白色 (R,G,B > 200)，将其 alpha 设为 0 (透明)
        if item[0] > 200 and item[1] > 200 and item[2] > 200:
            new_data.append((255, 255, 255, 0))
        else:
            new_data.append(item)

    img.putdata(new_data)
    img.save(output_path, "PNG")

make_transparent("A_H.jpg", "A_H_transparent.png")
