import pandas as pd

# df = pd.read_parquet("/home/dm/.cache/huggingface/lerobot/folding/eval_unfold_240/data/chunk-000/episode_000000.parquet", columns=["action", "observation.state"])
# ts = pd.read_parquet("/home/yixuan/.cache/huggingface/lerobot/folding_1/data/chunk-000/episode_000011.parquet", columns=["timestamp"])

# new_df = df.map(lambda x: x[6])
# new_df["timestamp"]=ts


# # df=df.apply(extraction)

# df.to_csv("output.csv",index=False)




# 4,6,12,18,19,20,21,24,26,36
# 35,37,39,40,43,50,51,54,


import os
import subprocess
from collections import OrderedDict
import sys

def batch_reencode(input_dir, output_dir):
    # 收集所有MP4文件路径（包括子目录）
    file_list = []
    for root, _, files in os.walk(input_dir):
        for f in files:
            if f.lower().endswith(".mp4"):
                input_path = os.path.join(root, f)
                rel_path = os.path.relpath(input_path, input_dir)
                output_path = os.path.join(output_dir, rel_path)
                file_list.append((input_path, output_path))

    # 单线程处理
    for input_path, output_path in file_list:
        ffmpeg_args = OrderedDict(
          [
              ("-r", str(25)),
              ("-i", str(input_path)),
              ("-vcodec", "libopenh264"),
              ("-pix_fmt", "yuv420p"),
          ]
        )

        ffmpeg_args["-g"] = str(2)
        ffmpeg_args["-crf"] = str(30)
        ffmpeg_args["-loglevel"] = str("error")

        ffmpeg_args = [item for pair in ffmpeg_args.items() for item in pair]

        ffmpeg_cmd = ["ffmpeg"] + ffmpeg_args + [str(output_path)]
        # redirect stdin to subprocess.DEVNULL to prevent reading random keyboard inputs from terminal
        subprocess.run(ffmpeg_cmd, check=True, stdin=subprocess.DEVNULL)

if __name__ == "__main__":
    # 示例用法
    input_folder = sys.argv[1]
    output_folder = sys.argv[2]
    
    batch_reencode(
        input_dir=input_folder,
        output_dir=output_folder
    )