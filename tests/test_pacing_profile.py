'''Tests for the PacingProfile CSV parser / time-indexed speed schedule.'''

import pytest

from apm.control.pacing_profile import PacingProfile, Segment, available_profiles


def test_parses_header_and_rows():
    '''The header line is skipped; data rows become segments.'''
    rows = [
        ['speed (m/s)', ' time (s)', ''],   # header with trailing comma
        ['0.5', '1'],
        ['1.0', '30'],
    ]
    p = PacingProfile.from_rows(rows)
    assert [(s.speed, s.duration) for s in p.segments] == [(0.5, 1.0), (1.0, 30.0)]
    assert p.total_duration == pytest.approx(31.0)


def test_skips_blanks_and_comments():
    rows = [['# warmup then cruise'], [], ['1.0', '5'], ['  ', ''], ['2.0', '5']]
    p = PacingProfile.from_rows(rows)
    assert len(p.segments) == 2


def test_speed_at_is_piecewise_constant():
    p = PacingProfile([Segment(0.5, 1.0), Segment(1.0, 2.0), Segment(3.0, 1.0)])
    assert p.speed_at(-1.0) == 0.5      # clamped to first segment
    assert p.speed_at(0.0) == 0.5
    assert p.speed_at(0.5) == 0.5
    assert p.speed_at(1.0) == 1.0       # boundary belongs to the next segment
    assert p.speed_at(2.9) == 1.0
    assert p.speed_at(3.0) == 3.0
    assert p.speed_at(99.0) == 3.0      # holds last segment past the end


def test_is_finished():
    p = PacingProfile([Segment(1.0, 2.0), Segment(2.0, 3.0)])
    assert p.total_duration == pytest.approx(5.0)
    assert not p.is_finished(4.99)
    assert p.is_finished(5.0)
    assert p.is_finished(10.0)


def test_from_csv_roundtrip(tmp_path):
    csv_file = tmp_path / 'profile.csv'
    csv_file.write_text('speed (m/s), time (s),\n0.5, 1\n1, 1\n', encoding='utf-8')
    p = PacingProfile.from_csv(csv_file)
    assert [(s.speed, s.duration) for s in p.segments] == [(0.5, 1.0), (1.0, 1.0)]


def test_empty_profile_raises():
    with pytest.raises(ValueError):
        PacingProfile.from_rows([['speed (m/s)', 'time (s)']])  # header only, no data


def test_malformed_data_row_raises():
    with pytest.raises(ValueError):
        PacingProfile.from_rows([['1.0', '5'], ['fast', 'soon']])


def test_negative_speed_raises():
    with pytest.raises(ValueError):
        PacingProfile.from_rows([['-1.0', '5']])


def test_non_positive_duration_raises():
    with pytest.raises(ValueError):
        PacingProfile.from_rows([['1.0', '0']])


def test_total_distance():
    p = PacingProfile([Segment(0.5, 1.0), Segment(1.0, 2.0)])  # 0.5*1 + 1.0*2
    assert p.total_distance == pytest.approx(2.5)


def test_distance_curve_is_cumulative_from_origin():
    p = PacingProfile([Segment(0.5, 2.0), Segment(2.0, 1.0)])
    times, dists = p.distance_curve()
    assert times == [0.0, 2.0, 3.0]
    assert dists == pytest.approx([0.0, 1.0, 3.0])   # 0, 0.5*2, +2.0*1


def test_speed_curve_steps_and_closes_at_end():
    p = PacingProfile([Segment(0.5, 2.0), Segment(2.0, 1.0)])
    times, speeds = p.speed_curve()
    assert times == [0.0, 2.0, 3.0]                  # closing point at total_duration
    assert speeds == [0.5, 2.0, 2.0]                 # last speed repeated to close the step


def test_available_profiles_lists_csvs(tmp_path):
    (tmp_path / 'b.csv').write_text('speed (m/s), time (s)\n1, 1\n', encoding='utf-8')
    (tmp_path / 'a.csv').write_text('speed (m/s), time (s)\n2, 1\n', encoding='utf-8')
    (tmp_path / 'notes.txt').write_text('ignore me', encoding='utf-8')
    found = available_profiles(tmp_path)
    assert [p.name for p in found] == ['a.csv', 'b.csv']   # sorted, CSVs only


def test_available_profiles_missing_dir_is_empty(tmp_path):
    assert available_profiles(tmp_path / 'does_not_exist') == []
