import logging
import time
from typing import Final

import attr

# Default rate-limit window for *_throttled calls. 1 line/sec keeps a sustained issue visible to an
# operator without flooding the log at kyber / talos / metis tick rates (often 100-500 Hz).
_DEFAULT_THROTTLE_INTERVAL_S: Final[float] = 1.0

# LogRecord extra-field names the rate-limiting filter inspects. Records without rate_limit_key
# pass through unchanged so non-throttled calls are unaffected.
_RATE_LIMIT_KEY_FIELD: Final[str] = "rate_limit_key"
_RATE_LIMIT_INTERVAL_FIELD: Final[str] = "rate_limit_interval_s"


class _RateLimitFilter(logging.Filter):
    """
    logging.Filter that drops log records carrying a rate_limit_key when
    another record with the same (logger_name, key) was emitted within
    rate_limit_interval_s. Records without rate_limit_key pass through.
    """

    def __init__(self) -> None:
        super().__init__()
        self._last_emit_s: dict[tuple[str, str], float] = {}

    def filter(self, record: logging.LogRecord) -> bool:
        key = getattr(record, _RATE_LIMIT_KEY_FIELD, None)
        if key is None:
            return True
        interval_s = getattr(record, _RATE_LIMIT_INTERVAL_FIELD, _DEFAULT_THROTTLE_INTERVAL_S)
        bucket = (record.name, str(key))
        now_s = time.monotonic()
        last_s = self._last_emit_s.get(bucket)
        if last_s is not None and now_s - last_s < interval_s:
            return False
        self._last_emit_s[bucket] = now_s
        return True

    def reset(self) -> None:
        """
        Clear all throttle state. Tests use this to isolate one test
        case's throttle ledger from the next; production code should
        not call it.
        """
        self._last_emit_s.clear()


# Single shared filter so the throttle ledger is consistent across ManorLogger instances. Two
# ManorLogger(name="X") objects address the same underlying Python logger via getLogger, and they
# need to share throttle state so the second instance can't bypass the rate limit set by the first.
_RATE_LIMIT_FILTER: Final[_RateLimitFilter] = _RateLimitFilter()


@attr.frozen
class ManorLogger:
    name: str
    level: int = logging.INFO

    _logger: logging.Logger = attr.ib(init=False)

    @_logger.default
    def _initialzie_logger(self) -> logging.Logger:
        logging.basicConfig(
            format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
            datefmt="%m/%d/%Y %I:%M:%S %p",
        )
        _l = logging.getLogger(self.name)
        _l.setLevel(self.level)
        # Attach the throttle filter once per logger. logging.getLogger returns the same instance for
        # the same name across calls, so we have to guard against re-adding (which would cause the
        # filter to run twice per record and double-count emissions in the ledger).
        if _RATE_LIMIT_FILTER not in _l.filters:
            _l.addFilter(_RATE_LIMIT_FILTER)
        return _l

    def debug(self, message: str) -> None:
        self._logger.debug(message)

    def info(self, message: str) -> None:
        self._logger.info(message)

    def warning(self, message: str) -> None:
        self._logger.warning(message)

    def error(self, message: str) -> None:
        self._logger.error(message)

    def critical(self, message: str) -> None:
        self._logger.critical(message)

    def info_throttled(self, message: str, key: str, interval_s: float = _DEFAULT_THROTTLE_INTERVAL_S) -> None:
        """
        Emit at INFO level only if the previous *_throttled call with the same (logger_name, key) is
        older than interval_s. Use for messages that fire every tick but only need to surface
        periodically -- e.g. "policy publishing dead air" warnings inside a 500 Hz controller.
        """
        self._logger.info(message, extra=self._throttle_extra(key, interval_s))

    def warning_throttled(self, message: str, key: str, interval_s: float = _DEFAULT_THROTTLE_INTERVAL_S) -> None:
        self._logger.warning(message, extra=self._throttle_extra(key, interval_s))

    def error_throttled(self, message: str, key: str, interval_s: float = _DEFAULT_THROTTLE_INTERVAL_S) -> None:
        self._logger.error(message, extra=self._throttle_extra(key, interval_s))

    @staticmethod
    def _throttle_extra(key: str, interval_s: float) -> dict[str, object]:
        return {_RATE_LIMIT_KEY_FIELD: key, _RATE_LIMIT_INTERVAL_FIELD: interval_s}
