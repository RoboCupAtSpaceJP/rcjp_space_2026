#!/usr/bin/env python3

import datetime
import os
import tempfile
import yaml
import rospy


def write_scores(scores_dict, log_file_path, trial_id=None):
    """スコアを YAML 形式で書き込む。

    trial_id=None のとき（初回）: タイムスタンプを生成し新規エントリを追加。trial_id を返す。
    trial_id 指定のとき（2回目以降）: 該当エントリを上書き更新。同じ trial_id を返す。
    """
    try:
        sd = scores_dict
        total = sum([sd['start_task'], sd['navigation_task'],
                     sd['search_task'], sd['docking_task'], sd['time_bonus']])

        # 既存データのロード
        if os.path.exists(log_file_path):
            with open(log_file_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        trials = data.get('trials', [])

        if trial_id is None:
            # 初回: タイムスタンプを生成して新規エントリ追加
            trial_id = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            entry = {
                'timestamp': trial_id,
                'trial_number': sd['trial_number'],
                'start_task': sd['start_task'],
                'navigation_task': sd['navigation_task'],
                'search_task': sd['search_task'],
                'docking_task': sd['docking_task'],
                'time_bonus': sd['time_bonus'],
                'final_score': total,
            }
            trials.append(entry)
            rospy.loginfo("Scores written (new trial, id=%s): %s", trial_id, log_file_path)
        else:
            # 2回目以降: 該当エントリを検索して上書き
            found = False
            for i, entry in enumerate(trials):
                if entry.get('timestamp') == trial_id:
                    trials[i] = {
                        'timestamp': trial_id,
                        'trial_number': sd['trial_number'],
                        'start_task': sd['start_task'],
                        'navigation_task': sd['navigation_task'],
                        'search_task': sd['search_task'],
                        'docking_task': sd['docking_task'],
                        'time_bonus': sd['time_bonus'],
                        'final_score': total,
                    }
                    found = True
                    break
            if not found:
                # 該当エントリなし → 新規追加
                trials.append({
                    'timestamp': trial_id,
                    'trial_number': sd['trial_number'],
                    'start_task': sd['start_task'],
                    'navigation_task': sd['navigation_task'],
                    'search_task': sd['search_task'],
                    'docking_task': sd['docking_task'],
                    'time_bonus': sd['time_bonus'],
                    'final_score': total,
                })
            rospy.loginfo("Scores updated (id=%s): %s", trial_id, log_file_path)

        data['trials'] = trials

        # Atomic Write: 一時ファイルに書き出し → rename
        dir_name = os.path.dirname(log_file_path)
        fd, tmp_path = tempfile.mkstemp(dir=dir_name, suffix='.yaml.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            os.rename(tmp_path, log_file_path)
        except Exception:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
            raise

        return trial_id

    except Exception as e:
        rospy.logerr("Error writing scores: %s — resetting trial_id to None for retry", str(e))
        return None
