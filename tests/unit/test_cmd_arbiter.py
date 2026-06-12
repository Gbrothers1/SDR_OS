from src.sdr_os.control.cmd_arbiter import CmdVelArbiter


def make():
    return CmdVelArbiter(operator_grace_s=1.0, owner_ttl_s=0.2)


def test_first_nonzero_ui_takes_ownership():
    a = make()
    d = a.evaluate("ui", 1, is_zero=False, now=10.0)
    assert d.accepted and d.apply_velocity and a.owner == "ui"


def test_mcp_owns_when_operator_idle():
    a = make()
    a.evaluate("ui", 1, is_zero=True, now=10.0)   # idle zeros only
    d = a.evaluate("mcp", 1, is_zero=False, now=11.5)
    assert d.apply_velocity and a.owner == "mcp"


def test_mcp_blocked_during_operator_grace():
    a = make()
    a.evaluate("ui", 1, is_zero=False, now=10.0)  # operator active
    d = a.evaluate("mcp", 1, is_zero=False, now=10.5)  # 0.5s later < 1.0s grace
    assert not d.apply_velocity and a.owner == "ui"


def test_ui_nonzero_steals_from_mcp_instantly():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    d = a.evaluate("ui", 1, is_zero=False, now=10.05)
    assert d.apply_velocity and a.owner == "ui"


def test_ui_idle_zeros_never_preempt_mcp():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    d = a.evaluate("ui", 1, is_zero=True, now=10.05)
    assert d.accepted and not d.apply_velocity and a.owner == "mcp"
    assert d.refresh_ttl  # idle stream still feeds safety TTL


def test_owner_zeros_apply():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    d = a.evaluate("mcp", 2, is_zero=True, now=10.05)
    assert d.apply_velocity  # owner's stop command goes through


def test_no_owner_zeros_apply():
    a = make()
    d = a.evaluate("ui", 1, is_zero=True, now=10.0)
    assert d.apply_velocity and a.owner is None


def test_per_source_seq_drop_stale():
    a = make()
    a.evaluate("mcp", 5, is_zero=False, now=10.0)
    d = a.evaluate("mcp", 4, is_zero=False, now=10.01)
    assert not d.accepted


def test_per_source_seq_reset_jump_accepted():
    a = make()
    a.evaluate("mcp", 500, is_zero=False, now=10.0)
    d = a.evaluate("mcp", 1, is_zero=False, now=10.01)  # new session
    assert d.accepted


def test_sources_do_not_share_seq():
    a = make()
    a.evaluate("ui", 9000, is_zero=True, now=10.0)
    d = a.evaluate("mcp", 1, is_zero=False, now=11.5)
    assert d.accepted and d.apply_velocity


def test_owner_expires_when_silent():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    assert a.expire_owner(now=10.1) is False     # still fresh
    assert a.expire_owner(now=10.31) is True     # >0.2s silent → released
    assert a.owner is None
    # subsequent ui zeros now apply (functional HOLD with tab open)
    d = a.evaluate("ui", 9001, is_zero=True, now=10.32)
    assert d.apply_velocity
