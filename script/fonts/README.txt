本目录用于随工程分发中文字体，无需在每台电脑上 apt / 安装系统字体。

默认文件（若随仓库提供）:
  NotoSansCJKsc-Regular.otf
  来源: https://github.com/notofonts/noto-cjk
  许可: SIL Open Font License 1.1（见上游仓库内 LICENSE）

若缺失，可在本目录下执行:
  curl -fsSL -O https://raw.githubusercontent.com/notofonts/noto-cjk/main/Sans/OTF/SimplifiedChinese/NotoSansCJKsc-Regular.otf

脚本 visualize_routing_map_realtime.py 会优先加载本目录中上述文件名；
也可放置其他 .otf / .ttf / .ttc，或通过 --cjk-font-file / 环境变量 VISUAL_TOOL_CJK_FONT 指定路径。
