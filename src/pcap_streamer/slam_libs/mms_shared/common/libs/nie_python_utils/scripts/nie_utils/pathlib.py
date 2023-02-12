# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

""" Extensions to the pathlib standard library """

__all__ = ['add_suffix']

from pathlib import Path


def add_suffix(path: Path, suffix: str):
    path = Path(path)
    return path.parent / (path.name + suffix)
