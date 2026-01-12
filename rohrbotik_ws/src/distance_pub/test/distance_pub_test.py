import pytest
from distance_pub.distance_pub import clamp_distance
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

"""
Test lassen sich im Terminal unter folgendem Befehl ausführen:
"python3 -m pytest src/distance_pub/test/distance_pub_test.py -v"
"""

def test_too_high():
    assert clamp_distance(5.0) == 1.0

def test_too_low():
    assert clamp_distance(0.05) == 0.2

def test_valid():
    assert clamp_distance(0.7) == 0.7

def test_edges():
    assert clamp_distance(0.2) == 0.2
    assert clamp_distance(1.0) == 1.0