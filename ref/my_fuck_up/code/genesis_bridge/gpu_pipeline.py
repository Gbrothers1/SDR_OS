"""GPU-accelerated frame encoding pipeline.

Provides NVJPEG (via torchvision) and optional NVENC H.264 encoding
with automatic CPU fallback when GPU capabilities are unavailable.
"""
import logging
from typing import Optional

import cv2
import numpy as np
import torch

from genesis_bridge.frame_protocol import (
    pack_frame_header, FrameType, FrameFlags, annexb_to_avcc
)

logger = logging.getLogger(__name__)

# Codec string to profile name mapping
CODEC_TO_PROFILE = {
    'avc1.42E01E': 'baseline',
    'avc1.4D401E': 'main',
    'avc1.64001E': 'high',
}


class GpuFramePipeline:
    """GPU-accelerated frame encoding with automatic CPU fallback.

    Capabilities are detected at initialization:
    - has_cuda: CUDA available
    - has_nvjpeg: torchvision JPEG encoding works on CUDA
    - has_nvenc: PyAV h264_nvenc codec available

    Usage:
        pipeline = GpuFramePipeline()
        jpeg_bytes = pipeline.encode_jpeg(rgb_tensor, quality=80)
    """

    def __init__(self):
        self.has_cuda = torch.cuda.is_available()
        self.device = torch.device("cuda" if self.has_cuda else "cpu")
        self.has_nvjpeg = self._detect_nvjpeg()
        self.has_nvenc = self._detect_nvenc()

        # H.264 encoder state (lazy init)
        self._h264_encoder = None
        self._h264_width = 0
        self._h264_height = 0
        self._h264_profile = 'baseline'
        self._force_keyframe = False

        logger.info(
            f"GpuFramePipeline initialized: device={self.device}, "
            f"nvjpeg={self.has_nvjpeg}, nvenc={self.has_nvenc}"
        )

    def _detect_nvjpeg(self) -> bool:
        """Detect if NVJPEG encoding is available via torchvision."""
        if not self.has_cuda:
            return False
        try:
            import torchvision.io
            test = torch.zeros(3, 8, 8, dtype=torch.uint8, device='cuda')
            torchvision.io.encode_jpeg(test, quality=80)
            return True
        except Exception as e:
            logger.debug(f"NVJPEG not available: {e}")
            return False

    def _detect_nvenc(self) -> bool:
        """Detect if NVENC H.264 encoding is available.

        Tries PyAV first, then falls back to system ffmpeg subprocess.
        Sets self._nvenc_backend to 'pyav' or 'subprocess' accordingly.
        """
        self._nvenc_backend = None

        # Try PyAV first
        try:
            import av
            if 'h264_nvenc' in av.codecs_available:
                test_encoder = av.CodecContext.create('h264_nvenc', 'w')
                test_encoder.width = 256
                test_encoder.height = 256
                test_encoder.pix_fmt = 'yuv420p'
                test_encoder.bit_rate = 1_000_000
                test_encoder.open()
                test_encoder.close()
                self._nvenc_backend = 'pyav'
                logger.info("NVENC available via PyAV")
                return True
        except Exception as e:
            logger.debug(f"PyAV NVENC not available: {e}")

        # Fall back to system ffmpeg subprocess
        try:
            import subprocess
            result = subprocess.run(
                ['ffmpeg', '-f', 'lavfi', '-i', 'testsrc=duration=0.1:size=256x256:rate=10',
                 '-c:v', 'h264_nvenc', '-f', 'null', '-'],
                capture_output=True, timeout=10
            )
            if result.returncode == 0:
                self._nvenc_backend = 'subprocess'
                logger.info("NVENC available via system ffmpeg subprocess")
                return True
        except Exception as e:
            logger.debug(f"System ffmpeg NVENC not available: {e}")

        return False

    def encode_jpeg(self, rgb_tensor: torch.Tensor, quality: int = 80) -> bytes:
        """Encode RGB tensor to JPEG bytes.

        Args:
            rgb_tensor: HWC uint8 tensor (RGB format) on any device
            quality: JPEG quality 1-100

        Returns:
            JPEG-encoded bytes
        """
        if self.has_nvjpeg and rgb_tensor.is_cuda:
            return self._encode_jpeg_nvjpeg(rgb_tensor, quality)
        else:
            return self._encode_jpeg_cpu(rgb_tensor, quality)

    def encode_jpeg_numpy(self, rgb_np: np.ndarray, quality: int = 80) -> bytes:
        """Encode RGB numpy array to JPEG bytes.

        Args:
            rgb_np: HWC uint8 numpy array (RGB format)
            quality: JPEG quality 1-100

        Returns:
            JPEG-encoded bytes
        """
        if self.has_nvjpeg:
            rgb_tensor = torch.from_numpy(rgb_np).to(self.device)
            return self._encode_jpeg_nvjpeg(rgb_tensor, quality)
        else:
            return self._encode_jpeg_cpu_numpy(rgb_np, quality)

    def _encode_jpeg_nvjpeg(self, rgb_tensor: torch.Tensor, quality: int) -> bytes:
        """Encode using NVJPEG via torchvision."""
        import torchvision.io
        chw = rgb_tensor.permute(2, 0, 1).contiguous()
        jpeg_tensor = torchvision.io.encode_jpeg(chw, quality=quality)
        return bytes(jpeg_tensor.cpu().numpy())

    def _encode_jpeg_cpu(self, rgb_tensor: torch.Tensor, quality: int) -> bytes:
        """Encode using CPU OpenCV (fallback)."""
        rgb_np = rgb_tensor.cpu().numpy() if rgb_tensor.is_cuda else rgb_tensor.numpy()
        return self._encode_jpeg_cpu_numpy(rgb_np, quality)

    def _encode_jpeg_cpu_numpy(self, rgb_np: np.ndarray, quality: int) -> bytes:
        """Encode numpy array using CPU OpenCV."""
        bgr_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        _, buf = cv2.imencode('.jpg', bgr_np, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def encode_h264(self, rgb_tensor: torch.Tensor, bitrate: int = 8_000_000) -> bytes:
        """Encode RGB tensor to H.264 bytes using NVENC.

        Args:
            rgb_tensor: HWC uint8 tensor (RGB format)
            bitrate: Target bitrate in bits/sec

        Returns:
            H.264-encoded bytes (may be empty if no keyframe yet)

        Raises:
            RuntimeError: If NVENC is not available
        """
        if not self.has_nvenc:
            raise RuntimeError("NVENC not available. Install PyAV with NVENC support.")

        import av

        frame_np = rgb_tensor.cpu().numpy()
        height, width = frame_np.shape[:2]

        if self._h264_encoder is None or width != self._h264_width or height != self._h264_height:
            if self._h264_encoder is not None:
                self._h264_encoder.close()

            self._h264_encoder = av.CodecContext.create('h264_nvenc', 'w')
            self._h264_encoder.width = width
            self._h264_encoder.height = height
            self._h264_encoder.pix_fmt = 'yuv420p'
            self._h264_encoder.bit_rate = bitrate
            self._h264_encoder.options = {'preset': 'llhp'}
            self._h264_encoder.open()

            self._h264_width = width
            self._h264_height = height
            logger.info(f"H.264 encoder initialized: {width}x{height} @ {bitrate/1e6:.1f} Mbps")

        av_frame = av.VideoFrame.from_ndarray(frame_np, format='rgb24')
        packets = self._h264_encoder.encode(av_frame)
        return b''.join(bytes(p) for p in packets)

    def encode_h264_au(
        self,
        rgb_array: np.ndarray,
        frame_id: int,
        profile: str = 'baseline',
        bitrate: int = 8_000_000
    ) -> Optional[bytes]:
        """Encode RGB array to H.264 access unit with frame header.

        Args:
            rgb_array: HWC uint8 numpy array (RGB format)
            frame_id: Monotonic frame counter
            profile: H.264 profile ('baseline', 'main', 'high')
            bitrate: Target bitrate in bits/sec

        Returns:
            Framed access unit (12-byte header + AVCC payload) or None if buffered
        """
        if not self.has_nvenc:
            raise RuntimeError("NVENC not available")

        if self._nvenc_backend == 'subprocess':
            return self._encode_h264_subprocess(rgb_array, frame_id, profile, bitrate)

        return self._encode_h264_pyav(rgb_array, frame_id, profile, bitrate)

    def _encode_h264_subprocess(
        self,
        rgb_array: np.ndarray,
        frame_id: int,
        profile: str = 'baseline',
        bitrate: int = 8_000_000
    ) -> Optional[bytes]:
        """Encode using system ffmpeg subprocess (fallback for older drivers)."""
        import subprocess

        height, width = rgb_array.shape[:2]

        # Force keyframe by including -force_key_frames
        force_kf = 'expr:1' if self._force_keyframe else 'expr:eq(n,0)'
        if self._force_keyframe:
            self._force_keyframe = False

        # Run ffmpeg to encode single frame
        cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-pixel_format', 'rgb24',
            '-video_size', f'{width}x{height}',
            '-framerate', '60',
            '-i', 'pipe:0',
            '-c:v', 'h264_nvenc',
            '-profile:v', profile,
            '-preset', 'p1',
            '-tune', 'll',
            '-rc', 'cbr',
            '-b:v', str(bitrate),
            '-maxrate', str(bitrate),
            '-bufsize', str(bitrate // 2),
            '-g', '60',
            '-bf', '0',
            '-force_key_frames', force_kf,
            '-f', 'h264',
            'pipe:1'
        ]

        try:
            result = subprocess.run(
                cmd,
                input=rgb_array.tobytes(),
                capture_output=True,
                timeout=1.0
            )
            if result.returncode != 0:
                logger.error(f"ffmpeg error: {result.stderr.decode()[:200]}")
                return None

            annexb_bytes = result.stdout
            if not annexb_bytes:
                return None

            avcc_bytes = annexb_to_avcc(annexb_bytes)

            # Detect keyframe (IDR NAL type 5)
            is_keyframe = b'\x00\x00\x00\x01\x65' in annexb_bytes or b'\x00\x00\x01\x65' in annexb_bytes
            flags = 0
            if is_keyframe or frame_id == 0:
                flags |= FrameFlags.KEYFRAME | FrameFlags.HAS_SPS_PPS

            header = pack_frame_header(
                frame_type=FrameType.H264,
                payload_len=len(avcc_bytes),
                frame_id=frame_id,
                flags=flags
            )
            return header + avcc_bytes

        except subprocess.TimeoutExpired:
            logger.warning("ffmpeg encoding timeout")
            return None
        except Exception as e:
            logger.error(f"ffmpeg encoding error: {e}")
            return None

    def _encode_h264_pyav(
        self,
        rgb_array: np.ndarray,
        frame_id: int,
        profile: str = 'baseline',
        bitrate: int = 8_000_000
    ) -> Optional[bytes]:
        """Encode using PyAV (for newer drivers with compatible NVENC API)."""
        import av

        height, width = rgb_array.shape[:2]

        # Reinit encoder if dimensions or profile changed
        if (self._h264_encoder is None or
            width != self._h264_width or
            height != self._h264_height or
            profile != self._h264_profile):

            if self._h264_encoder is not None:
                self._h264_encoder.close()

            self._h264_encoder = av.CodecContext.create('h264_nvenc', 'w')
            self._h264_encoder.width = width
            self._h264_encoder.height = height
            self._h264_encoder.pix_fmt = 'yuv420p'
            self._h264_encoder.bit_rate = bitrate
            self._h264_encoder.options = {
                'profile': profile,
                'level': '3.0',
                'preset': 'p1',
                'tune': 'll',
                'rc': 'cbr',
                'maxrate': str(bitrate),
                'bufsize': str(bitrate // 2),
                'rc-lookahead': '0',
                'bf': '0',
                'g': '60',
                'repeat-headers': '1',
            }
            self._h264_encoder.open()

            self._h264_width = width
            self._h264_height = height
            self._h264_profile = profile
            self._force_keyframe = False
            logger.info(f"H.264 encoder initialized: {width}x{height} {profile}")

        # Handle force keyframe
        if self._force_keyframe:
            self._h264_encoder.close()
            self._h264_encoder = None
            self._force_keyframe = False
            return self._encode_h264_pyav(rgb_array, frame_id, profile, bitrate)

        # Encode frame
        av_frame = av.VideoFrame.from_ndarray(rgb_array, format='rgb24')
        av_frame.pts = frame_id

        packets = self._h264_encoder.encode(av_frame)

        if not packets:
            return None  # Encoder is buffering

        # Concatenate packets into access unit
        annexb_bytes = b''.join(bytes(p) for p in packets)
        avcc_bytes = annexb_to_avcc(annexb_bytes)

        # Determine flags
        is_keyframe = packets[0].is_keyframe
        flags = 0
        if is_keyframe:
            flags |= FrameFlags.KEYFRAME | FrameFlags.HAS_SPS_PPS
        # annexb=0 since we converted to AVCC

        # Build header + payload
        header = pack_frame_header(
            frame_type=FrameType.H264,
            payload_len=len(avcc_bytes),
            frame_id=frame_id,
            flags=flags
        )

        return header + avcc_bytes

    def request_keyframe(self):
        """Request next frame to be a keyframe (for new client joins)."""
        self._force_keyframe = True

    def close(self):
        """Clean up encoder resources."""
        if self._h264_encoder is not None:
            try:
                self._h264_encoder.close()
            except Exception:
                pass
            self._h264_encoder = None
