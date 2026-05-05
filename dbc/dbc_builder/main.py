import os
import cantools
import canboard

base_id = 1648

if __name__ == "__main__":
    db = canboard.build_db(base_id)

    script_dir = os.path.dirname(os.path.realpath(__file__)) #<-- absolute dir the script is in
    print(script_dir)
    rel_path = "../" + db.name + "_" + db.version + ".dbc"
    abs_file_path = os.path.abspath(os.path.realpath(os.path.join(script_dir, rel_path)))
    print(abs_file_path)
    with open(abs_file_path, 'w') as f:
        f.write(db.as_dbc_string()) 
    