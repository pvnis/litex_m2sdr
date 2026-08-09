#!/usr/bin/env python3
"""Account-neutral SSH/SCP transport for the Pavonis role hosts."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys
import tomllib
from typing import Sequence


@dataclass(frozen=True)
class Transport:
    known_hosts: Path
    strict_host_key_checking: str
    connect_timeout_seconds: int
    password_file: Path | None


@dataclass(frozen=True)
class Role:
    name: str
    host: str
    user: str
    home: str
    identity_file: Path | None
    port: int


@dataclass(frozen=True)
class Site:
    path: Path
    transport: Transport
    roles: dict[str, Role]
    raw: dict


def _expand(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def site_path(explicit: str | None = None) -> Path:
    value = explicit or os.environ.get("PAVONIS_SITE_CONFIG")
    if not value:
        raise ValueError("set --site or PAVONIS_SITE_CONFIG")
    path = _expand(value)
    if not path.is_file():
        raise ValueError(f"site config is not a file: {path}")
    return path


def _private_file(path: Path, label: str) -> None:
    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077:
        raise ValueError(f"{label} must not be group/world accessible: {path}")


def load_site(explicit: str | None = None) -> Site:
    path = site_path(explicit)
    _private_file(path, "site config")
    with path.open("rb") as handle:
        raw = tomllib.load(handle)

    source = raw.get("transport", {})
    policy = str(source.get("strict_host_key_checking", "yes"))
    if policy not in {"yes", "accept-new"}:
        raise ValueError("strict_host_key_checking must be yes or accept-new")
    password_value = str(source.get("password_file", "")).strip()
    password_file = _expand(password_value) if password_value else None
    if password_file is not None:
        if not password_file.is_file():
            raise ValueError(f"password file is not a file: {password_file}")
        _private_file(password_file, "password file")
    transport = Transport(
        known_hosts=_expand(str(source.get("known_hosts", "~/.ssh/known_hosts"))),
        strict_host_key_checking=policy,
        connect_timeout_seconds=int(source.get("connect_timeout_seconds", 8)),
        password_file=password_file,
    )

    roles: dict[str, Role] = {}
    for name in ("ran", "ue"):
        values = raw.get("roles", {}).get(name, {})
        host = str(values.get("host", ""))
        user = str(values.get("user", ""))
        home = str(values.get("home", ""))
        if not host or host.startswith("REPLACE_") or not user or not home.startswith("/"):
            raise ValueError(f"roles.{name} is incomplete")
        identity_value = str(values.get("identity_file", "")).strip()
        identity_file = _expand(identity_value) if identity_value else None
        if identity_file is not None and not identity_file.is_file():
            raise ValueError(f"identity file is not a file: {identity_file}")
        roles[name] = Role(
            name=name,
            host=host,
            user=user,
            home=home.rstrip("/"),
            identity_file=identity_file,
            port=int(values.get("port", 22)),
        )
    if roles["ran"].home != roles["ue"].home:
        raise ValueError("roles.ran.home and roles.ue.home must match")
    return Site(path=path, transport=transport, roles=roles, raw=raw)


def role(site: Site, name: str) -> Role:
    aliases = {"ran": "ran", "ue": "ue"}
    try:
        return site.roles[aliases[name]]
    except KeyError as exc:
        raise ValueError(f"unknown role {name!r}; expected ran or ue") from exc


def ssh_options(site: Site, target: Role) -> list[str]:
    options = [
        "-o", f"StrictHostKeyChecking={site.transport.strict_host_key_checking}",
        "-o", f"UserKnownHostsFile={site.transport.known_hosts}",
        "-o", "LogLevel=ERROR",
        "-o", f"ConnectTimeout={site.transport.connect_timeout_seconds}",
        "-p", str(target.port),
    ]
    if target.identity_file is not None:
        options += ["-i", str(target.identity_file), "-o", "IdentitiesOnly=yes"]
    return options


def _run(
    argv: Sequence[str],
    output: Path,
    timeout: int,
    password_file: Path | None,
) -> int:
    output.parent.mkdir(parents=True, exist_ok=True)
    if password_file is None:
        with output.open("w", encoding="utf-8", errors="replace") as handle:
            try:
                result = subprocess.run(
                    argv,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=timeout,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                handle.write("\nTRANSPORT_TIMEOUT\n")
                return 124
        return result.returncode

    try:
        import pexpect
    except ImportError:
        print("pexpect is required for password authentication", file=sys.stderr)
        return 69
    password = password_file.read_text(encoding="utf-8").splitlines()[0]
    child = pexpect.spawn(argv[0], list(argv[1:]), encoding="utf-8", timeout=timeout)
    with output.open("w", encoding="utf-8", errors="replace") as handle:
        child.logfile_read = handle
        while True:
            index = child.expect([r"(?i)password:", pexpect.EOF, pexpect.TIMEOUT])
            if index == 0:
                child.sendline(password)
                continue
            if index == 1:
                break
            handle.write("\nTRANSPORT_TIMEOUT\n")
            child.close(force=True)
            return 124
    child.close()
    if child.exitstatus is None:
        return 1 if child.signalstatus else 0
    return int(child.exitstatus)


def run_ssh(site: Site, target: Role, command: Sequence[str], output: Path, timeout: int) -> int:
    remote = shlex.join(command)
    argv = ["ssh", *ssh_options(site, target), f"{target.user}@{target.host}", remote]
    return _run(argv, output, timeout, site.transport.password_file)


def run_scp(
    site: Site,
    target: Role,
    source: str,
    destination: str,
    output: Path,
    timeout: int,
    direction: str,
) -> int:
    options = ssh_options(site, target)
    port_index = options.index("-p")
    options[port_index] = "-P"
    remote = f"{target.user}@{target.host}"
    if direction == "put":
        argv = ["scp", *options, source, f"{remote}:{destination}"]
    elif direction == "get":
        argv = ["scp", *options, f"{remote}:{source}", destination]
    else:
        raise ValueError(f"invalid direction: {direction}")
    return _run(argv, output, timeout, site.transport.password_file)

