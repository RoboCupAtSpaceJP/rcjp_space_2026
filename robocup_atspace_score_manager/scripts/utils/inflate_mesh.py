#!/usr/bin/env python3
import os
import sys
import functools

# --- Python 3.8 互換性パッチ ---
if sys.version_info < (3, 9):
    functools.cache = functools.lru_cache(maxsize=None)

import rospy
import rospkg
import trimesh
import numpy as np

def inflate_dae(model_name="float2_blue", offset=0.06):
    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path('robocup_atspace_score_manager')
    except Exception:
        print("Error: Could not find package path with rospack.")
        return None
    
    input_path = os.path.join(pkg_path, 'models', model_name, 'meshes', f"{model_name}.dae")
    output_dir = os.path.join(pkg_path, 'models', 'generated_meshes', model_name)
    
    output_filename = f"inflated_{int(offset*100)}cm.stl"
    output_path = os.path.join(output_dir, output_filename)

    # 既存チェックを一旦外して確実に上書き実行させます
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"--- Processing Model: {model_name} ---")
    print(f"Loading: {input_path}")
    
    try:
        scene_or_mesh = trimesh.load(input_path)
    except Exception as e:
        print(f"Load Error: {e}")
        return None

    # Meshの統合
    if isinstance(scene_or_mesh, trimesh.Scene):
        mesh = scene_or_mesh.to_mesh() if hasattr(scene_or_mesh, 'to_mesh') else \
               trimesh.util.concatenate([g for g in scene_or_mesh.geometry.values() if isinstance(g, trimesh.Trimesh)])
    else:
        mesh = scene_or_mesh

    # --- 法線膨張ロジック ---
    print(f"Generating inflated mesh (Normal Offset +{offset}m)...")
    
    # 1. 形状をコピー
    macho_mesh = mesh.copy()

    # 2. 各頂点を「頂点法線」の方向にオフセット
    # エラー回避のため、簡略化処理は行わずにそのまま膨らませます
    macho_mesh.vertices += macho_mesh.vertex_normals * offset

    # 3. 保存
    print(f"Exporting: {output_path}")
    macho_mesh.export(output_path)
    
    return output_path

if __name__ == "__main__":
    # offsetを 0.6 (60cm) に設定して実行
    try:
        res = inflate_dae("float2_blue", 0.6)
        if res:
            print(f"Done: {os.path.basename(res)}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")