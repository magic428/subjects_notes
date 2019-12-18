#!coding=utf-8

import os
import os.path

def get_file_title(file_path):
    if not os.path.exists(file_path):
        print("**error** No Such File: ", file_path)

    title = ''  
    with open(file_path,'r', encoding='utf-8') as f:
        lines = f.readlines()
        for line in lines:
            title = line.replace('#', '')
            title = title.strip()
            if( 0 != len(title) ):
                break
    return title

if __name__ == "__main__":    
    # this folder is custom 
    i = 0
    rootdir=u"/home/magic/work/gitwork/work_note/ubuntu_os_usage"

    level_1_folers = os.listdir(rootdir)
    for level_1_foler in level_1_folers:
        i += 1
        if (level_1_foler[0] == '.'):
            continue

        level_2_dir = os.path.join(rootdir, level_1_foler)  # 第二层目录
        file_title = []
        top_readme_filepath = ''
        top_readme_title = ''
        if os.path.isdir(level_2_dir):
            level_2_files = os.listdir(level_2_dir)
            
            # get the README.md in the top-2 level
            for level_2_filename in level_2_files:
                
                upcase_level_2_filename = level_2_filename.upper()
                level_2_filename = os.path.join(level_2_dir, level_2_filename)
                if -1 != upcase_level_2_filename.find("README.MD"):
                    top_readme_filepath = level_2_filename
                    top_readme_title = get_file_title(level_2_filename)
                    print("real: ", top_readme_title)
            
            first_open = True
            file_content_append_catogray = False
            for level_2_filename in level_2_files:
                print(level_2_filename)
                upcase_level_2_filename = level_2_filename.upper()
                level_2_filename = os.path.join(level_2_dir, level_2_filename)

                # get all subdir *.md files under level-2 dirs
                if(os.path.isdir(level_2_filename)):
                    for parent,dirnames,filenames in os.walk(level_2_filename):
                        file_content = []
                        #case 1:
                        # for dirname in dirnames:
                            # print("parent folder is:" + parent)
                            # print("dirname is:" + dirname)
                        #case 2
                        sub_title = '\n## ' + os.path.basename(parent)+'\n\n'
                        if sub_title not in file_content:
                            file_content.append(sub_title)

                        for filename in filenames:	
                            # print("parent folder is:" + parent)
                            # print("filename with full path:"+ os.path.join(parent,filename))
                            # Get sub-content title
                            filepath = os.path.join(parent,filename)
                            # if -1 != filepath.upper().find("README.MD") and filepath != top_readme_filepath:
                                # file_title.append(get_file_title(filepath))
                            if filepath == top_readme_filepath:
                                continue

                            if -1 != filepath.upper().find(".MD"):
                                blog_title = get_file_title(filepath)
                                https_path = 'https://www.github.com/magic428/work_note/blob/master' + (filepath[filepath.find("work_note")+len("work_note"):]).replace(os.sep, '/')
                                item = '![{}]({})'.format(blog_title, https_path)
                                file_content.append(item+'\n')
                        if(len(file_content) > 1) and os.path.exists(top_readme_filepath):
                            print(file_content)
                            if first_open:
                                first_open = False
                                with open(top_readme_filepath,'w', encoding='utf-8') as f:
                                    f.write("# " + top_readme_title + '\n')
                                    for line in file_content:
                                        f.write(line)
                            else:
                                with open(top_readme_filepath,'a', encoding='utf-8') as f:
                                    for line in file_content:
                                        f.write(line)

                elif -1 != upcase_level_2_filename.find(".MD") and os.path.exists(top_readme_filepath):
                    blog_title = get_file_title(level_2_filename)
                    https_path = 'https://www.github.com/magic428/work_note/blob/master' + (level_2_filename[level_2_filename.find("work_note")+len("work_note"):]).replace(os.sep, '/')
                    item = '![{}]({})'.format(blog_title, https_path) + '\n'
                    if first_open:
                        first_open = False
                        with open(top_readme_filepath,'w', encoding='utf-8') as f:
                            f.write("# " + top_readme_title + '\n')
                            f.write('\n## Introduce\n\n')
                            f.write(item)

                    else:
                        with open(top_readme_filepath,'a', encoding='utf-8') as f:
                            f.write(item)

        if i >= 14:
            break
