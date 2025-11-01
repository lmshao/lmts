# lmts - MPEG-TS 流处理库

一个用于 MPEG 传输流（TS）复用和解复用操作的 C++17 库。

## 功能特性

- **TS 复用**: 将音视频流编码为 MPEG-TS 格式
- **TS 解复用**: 解析并提取 MPEG-TS 数据中的流
- **流格式支持**: H.264/H.265 视频，AAC/AC3 音频
- **简洁设计**: 结构清晰的 TS 协议实现

## 依赖项

- **lmcore**: 核心工具和数据结构
- **C++17**: 现代 C++ 标准
- **CMake 3.10+**: 构建系统

## 构建

```bash
# 配置
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 构建
cmake --build build

# 安装（可选）
cmake --install build
```

## 使用示例

### TS 复用

```cpp
#include <lmts/ts_muxer.h>

using namespace lmshao::lmts;

auto muxer = std::make_unique<TSMuxer>();
muxer->SetVideoPID(0x100);
muxer->SetAudioPID(0x101);
muxer->Start();

// 复用视频数据
muxer->MuxVideoData(video_data, video_size, timestamp);

// 复用音频数据
muxer->MuxAudioData(audio_data, audio_size, timestamp);
```

### TS 解复用

```cpp
#include <lmts/ts_demuxer.h>

using namespace lmshao::lmts;

auto demuxer = std::make_unique<TSDemuxer>();
demuxer->SetCallback(callback);
demuxer->Start();

// 解析 TS 数据
demuxer->ParseData(ts_data, ts_size);
```

## 许可证

MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 作者

SHAO Liming <lmshao@163.com>