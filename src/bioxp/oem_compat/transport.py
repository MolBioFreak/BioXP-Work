from __future__ import annotations

from dataclasses import dataclass, field

from .frames import OemCommandFrame, OemReplyFrame


class MissingDryRunReply(RuntimeError):
    pass


@dataclass
class DryRunTransport:
    """No-hardware transport for OEM compatibility oracle tests.

    It records the frames that would have been sent through the OEM CAN transport.
    By default it returns a synthetic success ACK so higher layers can be exercised
    on the workstation. Strict oracle tests can require configured replies.
    """

    require_configured_replies: bool = False
    configured_replies: dict[OemCommandFrame, OemReplyFrame] = field(default_factory=dict)
    frames: list[OemCommandFrame] = field(default_factory=list)
    opened_usb: bool = False

    def transmit(self, frame: OemCommandFrame) -> OemReplyFrame:
        self.frames.append(frame)
        if frame in self.configured_replies:
            return self.configured_replies[frame]
        if self.require_configured_replies:
            raise MissingDryRunReply(f"No dry-run reply configured for {frame!r}")
        return OemReplyFrame(category=frame.category, status=100, synthetic=True)
