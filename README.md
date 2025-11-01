# lmts - MPEG-TS Stream Processing Library

A C++17 library for MPEG Transport Stream (TS) muxing and demuxing operations.

## Features

- **TS Muxing**: Encode audio/video streams into MPEG-TS format
- **TS Demuxing**: Parse and extract streams from MPEG-TS data
- **Stream Support**: H.264/H.265 video, AAC/AC3 audio
- **Clean Design**: Well-structured code for TS protocol implementation

## Dependencies

- **lmcore**: Core utilities and data structures
- **C++17**: Modern C++ standard support
- **CMake 3.10+**: Build system

## Building

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build

# Install (optional)
cmake --install build
```

## Usage

### TS Muxing

```cpp
#include <lmts/ts_muxer.h>

using namespace lmshao::lmts;

auto muxer = std::make_unique<TSMuxer>();
muxer->SetVideoPID(0x100);
muxer->SetAudioPID(0x101);
muxer->Start();

// Mux video data
muxer->MuxVideoData(video_data, video_size, timestamp);

// Mux audio data  
muxer->MuxAudioData(audio_data, audio_size, timestamp);
```

### TS Demuxing

```cpp
#include <lmts/ts_demuxer.h>

using namespace lmshao::lmts;

auto demuxer = std::make_unique<TSDemuxer>();
demuxer->SetCallback(callback);
demuxer->Start();

// Parse TS data
demuxer->ParseData(ts_data, ts_size);
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Author

SHAO Liming <lmshao@163.com>