"""Per-client stream fanout with backpressure queues."""
import asyncio
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


@dataclass
class ClientStream:
    """Single client's stream with bounded queue."""

    client_id: str
    websocket: Any  # WebSocket connection
    supports_h264: bool
    h264_codec: str = "avc1.42E01E"
    queue: asyncio.Queue = field(default_factory=lambda: asyncio.Queue(maxsize=2))
    _task: Optional[asyncio.Task] = field(default=None, repr=False)
    _running: bool = field(default=False, repr=False)

    def enqueue(self, frame_bytes: bytes):
        """Add frame to queue, dropping oldest if full."""
        while self.queue.full():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        try:
            self.queue.put_nowait(frame_bytes)
        except asyncio.QueueFull:
            pass

    async def start(self):
        """Start the send loop."""
        self._running = True
        try:
            while self._running:
                try:
                    frame_bytes = await asyncio.wait_for(
                        self.queue.get(), timeout=1.0
                    )
                    await self.websocket.send(frame_bytes)
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    logger.debug(f"Client {self.client_id} send error: {e}")
                    break
        finally:
            try:
                await self.websocket.close()
            except Exception:
                pass

    def stop(self):
        """Signal send loop to stop."""
        self._running = False

    def is_alive(self) -> bool:
        """Check if send task is still running."""
        return self._task is not None and not self._task.done()


class StreamFanout:
    """Manages per-client streams with codec-based grouping."""

    def __init__(self):
        self.h264_clients: Dict[str, ClientStream] = {}
        self.jpeg_clients: Dict[str, ClientStream] = {}
        self._force_keyframe: bool = False

    def add_client(
        self,
        client_id: str,
        websocket,
        supports_h264: bool,
        h264_codec: str = "avc1.42E01E"
    ):
        """Add client to appropriate fanout group."""
        client = ClientStream(
            client_id=client_id,
            websocket=websocket,
            supports_h264=supports_h264,
            h264_codec=h264_codec
        )

        client._task = asyncio.create_task(client.start())

        if supports_h264:
            self.h264_clients[client_id] = client
            self._force_keyframe = True
            logger.info(f"Added H.264 client: {client_id} ({h264_codec})")
        else:
            self.jpeg_clients[client_id] = client
            logger.info(f"Added JPEG client: {client_id}")

    async def remove_client(self, client_id: str):
        """Remove client from fanout."""
        for clients in [self.h264_clients, self.jpeg_clients]:
            if client_id in clients:
                client = clients.pop(client_id)
                client.stop()
                if client._task:
                    try:
                        await asyncio.wait_for(client._task, timeout=1.0)
                    except asyncio.TimeoutError:
                        client._task.cancel()
                logger.info(f"Removed client: {client_id}")
                return

    def broadcast_h264(self, frame_bytes: bytes):
        """Send H.264 frame to all H.264 clients."""
        dead_clients = []
        for cid, client in self.h264_clients.items():
            if not client.is_alive():
                dead_clients.append(cid)
            else:
                client.enqueue(frame_bytes)

        for cid in dead_clients:
            del self.h264_clients[cid]
            logger.debug(f"Pruned dead H.264 client: {cid}")

    def broadcast_jpeg(self, frame_bytes: bytes):
        """Send JPEG frame to all JPEG clients."""
        dead_clients = []
        for cid, client in self.jpeg_clients.items():
            if not client.is_alive():
                dead_clients.append(cid)
            else:
                client.enqueue(frame_bytes)

        for cid in dead_clients:
            del self.jpeg_clients[cid]
            logger.debug(f"Pruned dead JPEG client: {cid}")

    def should_force_keyframe(self) -> bool:
        """Check and clear force keyframe flag."""
        if self._force_keyframe:
            self._force_keyframe = False
            return True
        return False

    def has_h264_clients(self) -> bool:
        return len(self.h264_clients) > 0

    def has_jpeg_clients(self) -> bool:
        return len(self.jpeg_clients) > 0

    async def close_all(self):
        """Close all client connections."""
        all_ids = list(self.h264_clients.keys()) + list(self.jpeg_clients.keys())
        for cid in all_ids:
            await self.remove_client(cid)
