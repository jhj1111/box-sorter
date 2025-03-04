import os
import shutil

def ensure_labels_folders(base_dirs, sub_dirs):
    for base_dir in base_dirs:
        for sub_dir in sub_dirs:
            labels_path = os.path.join(base_dir, sub_dir, "labels")
            os.makedirs(labels_path, exist_ok=True)

def copy_matching_labels(base_dirs, sub_dirs):
    for base_dir in base_dirs:
        for sub_dir in sub_dirs:
            images_path = os.path.join(base_dir, sub_dir, "images")
            labels_path = os.path.join(base_dir, sub_dir, "labels")
            label_txt_path = os.path.join("label_txt")
            
            if not os.path.exists(images_path) or not os.path.exists(label_txt_path):
                continue
            
            image_files = {os.path.splitext(f)[0] for f in os.listdir(images_path) if f.endswith(".jpg")}
            label_files = {f for f in os.listdir(label_txt_path) if f.endswith(".txt")}
            
            matching_files = image_files.intersection({os.path.splitext(f)[0] for f in label_files})
            
            for file in matching_files:
                src_file = os.path.join(label_txt_path, file + ".txt")
                dst_file = os.path.join(labels_path, file + ".txt")
                shutil.copy2(src_file, dst_file)
                print(f"Copied: {src_file} -> {dst_file}")

if __name__ == "__main__":
    base_dirs = ["aruco_dataset", "blue_dataset", "red_dataset", "box_dataset"]
    sub_dirs = ["test", "train", "valid"]
    
    ensure_labels_folders(base_dirs, sub_dirs)
    copy_matching_labels(base_dirs, sub_dirs)
