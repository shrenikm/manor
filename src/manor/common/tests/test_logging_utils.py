"""
Tests for ManorLogger -- specifically the rate-limited *_throttled
methods built on top of a logging.Filter. The pass-through (info /
warning / error) methods are thin shims around stdlib logging and
aren't worth covering directly.

The shared _RATE_LIMIT_FILTER carries throttle state across instances,
so tests reset() it in a fixture to keep cases independent.
"""

from __future__ import annotations

import logging

import pytest

from manor.common import logging_utils
from manor.common.logging_utils import ManorLogger
from manor.common.testing_utils import run_manor_tests


@pytest.fixture(autouse=True)
def reset_throttle_filter() -> None:
    logging_utils._RATE_LIMIT_FILTER.reset()


def _unique_logger_name(test_name: str) -> str:
    # Each test gets its own underlying Python logger so that the (logger_name, key) bucket the
    # filter uses doesn't accidentally bleed across cases that share a key.
    return f"test_logging_utils.{test_name}"


class TestRateLimitFilter:
    def test_records_without_key_pass_through(self, caplog: pytest.LogCaptureFixture) -> None:
        # The filter must be a no-op for normal logger calls -- only records carrying
        # rate_limit_key in extras are eligible for throttling.
        logger = ManorLogger(name=_unique_logger_name("pass_through"))
        with caplog.at_level(logging.WARNING, logger=logger.name):
            logger.warning("first")
            logger.warning("second")
            logger.warning("third")
        emitted = [r.getMessage() for r in caplog.records if r.name == logger.name]
        assert emitted == ["first", "second", "third"]

    def test_throttled_suppresses_within_window(
        self,
        caplog: pytest.LogCaptureFixture,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # Drive the filter's clock so the test doesn't have to sleep. First call passes; subsequent
        # calls within the window are dropped.
        fake_now = {"t": 100.0}
        monkeypatch.setattr(logging_utils.time, "monotonic", lambda: fake_now["t"])

        logger = ManorLogger(name=_unique_logger_name("suppress"))
        with caplog.at_level(logging.WARNING, logger=logger.name):
            logger.warning_throttled("msg", key="k", interval_s=1.0)
            fake_now["t"] += 0.5
            logger.warning_throttled("msg", key="k", interval_s=1.0)
            fake_now["t"] += 0.49
            logger.warning_throttled("msg", key="k", interval_s=1.0)
        emitted = [r.getMessage() for r in caplog.records if r.name == logger.name]
        assert emitted == ["msg"]

    def test_throttled_emits_again_after_window(
        self,
        caplog: pytest.LogCaptureFixture,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        fake_now = {"t": 100.0}
        monkeypatch.setattr(logging_utils.time, "monotonic", lambda: fake_now["t"])

        logger = ManorLogger(name=_unique_logger_name("after_window"))
        with caplog.at_level(logging.WARNING, logger=logger.name):
            logger.warning_throttled("first", key="k", interval_s=0.5)
            fake_now["t"] += 0.6
            logger.warning_throttled("second", key="k", interval_s=0.5)
        emitted = [r.getMessage() for r in caplog.records if r.name == logger.name]
        assert emitted == ["first", "second"]

    def test_distinct_keys_have_independent_throttles(
        self,
        caplog: pytest.LogCaptureFixture,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # A warning under key "alpha" must not suppress a warning under key "beta" -- distinct
        # call sites that happen to fire at the same time should both be heard.
        fake_now = {"t": 100.0}
        monkeypatch.setattr(logging_utils.time, "monotonic", lambda: fake_now["t"])

        logger = ManorLogger(name=_unique_logger_name("distinct_keys"))
        with caplog.at_level(logging.WARNING, logger=logger.name):
            logger.warning_throttled("alpha-msg", key="alpha", interval_s=1.0)
            logger.warning_throttled("beta-msg", key="beta", interval_s=1.0)
            logger.warning_throttled("alpha-msg-2", key="alpha", interval_s=1.0)
        emitted = [r.getMessage() for r in caplog.records if r.name == logger.name]
        assert emitted == ["alpha-msg", "beta-msg"]

    def test_distinct_logger_names_have_independent_throttles(
        self,
        caplog: pytest.LogCaptureFixture,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # Two ManorLogger instances on different names sharing a key must not throttle each other.
        # The filter buckets state by (logger_name, key).
        fake_now = {"t": 100.0}
        monkeypatch.setattr(logging_utils.time, "monotonic", lambda: fake_now["t"])

        a_name = _unique_logger_name("isolation_a")
        b_name = _unique_logger_name("isolation_b")
        logger_a = ManorLogger(name=a_name)
        logger_b = ManorLogger(name=b_name)
        with caplog.at_level(logging.WARNING):
            logger_a.warning_throttled("from-a", key="shared", interval_s=1.0)
            logger_b.warning_throttled("from-b", key="shared", interval_s=1.0)
        emitted_a = [r.getMessage() for r in caplog.records if r.name == a_name]
        emitted_b = [r.getMessage() for r in caplog.records if r.name == b_name]
        assert emitted_a == ["from-a"]
        assert emitted_b == ["from-b"]

    def test_two_instances_same_name_share_throttle_state(
        self,
        caplog: pytest.LogCaptureFixture,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # Two ManorLogger(name="X") instances address the same underlying Python logger via
        # getLogger, and they must share throttle state so a second instance can't bypass the rate
        # limit set by the first.
        fake_now = {"t": 100.0}
        monkeypatch.setattr(logging_utils.time, "monotonic", lambda: fake_now["t"])

        name = _unique_logger_name("shared_state")
        first = ManorLogger(name=name)
        second = ManorLogger(name=name)
        with caplog.at_level(logging.WARNING, logger=name):
            first.warning_throttled("via-first", key="k", interval_s=1.0)
            second.warning_throttled("via-second", key="k", interval_s=1.0)
        emitted = [r.getMessage() for r in caplog.records if r.name == name]
        assert emitted == ["via-first"]

    def test_info_and_error_throttled_share_window_per_key(
        self,
        caplog: pytest.LogCaptureFixture,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # The throttle is keyed on (logger_name, key); the level is not part of the bucket. Mixing
        # info_throttled / warning_throttled / error_throttled under the same key shares a window,
        # which matches the intent ("rate-limit messages about thing X").
        fake_now = {"t": 100.0}
        monkeypatch.setattr(logging_utils.time, "monotonic", lambda: fake_now["t"])

        logger = ManorLogger(name=_unique_logger_name("levels"))
        with caplog.at_level(logging.DEBUG, logger=logger.name):
            logger.info_throttled("info-msg", key="k", interval_s=1.0)
            logger.warning_throttled("warn-msg", key="k", interval_s=1.0)
            logger.error_throttled("error-msg", key="k", interval_s=1.0)
        emitted = [(r.levelname, r.getMessage()) for r in caplog.records if r.name == logger.name]
        assert emitted == [("INFO", "info-msg")]


if __name__ == "__main__":
    run_manor_tests()
