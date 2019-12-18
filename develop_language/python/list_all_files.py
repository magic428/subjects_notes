#!coding=utf-8
# 输出一个文件夹下所有文件
import os
import re

def get_file_title(file_path):
    if not os.path.exists(file_path):
        print("**error** No Such File: ", file_path)

    title = ''  
    with open(file_path,'r', encoding='utf-8') as f:
        lines = f.readlines()
        for line in lines:
            if not line.startswith('# '):
                continue
            title = line.replace('#', '')
            title = title.strip()
            if( 0 != len(title) ):
                break
    return title

def filterFileName(filename):
    # if re.search('.doc\w{0,1}',filename,re.I)!=None:
    if re.search('.md',filename,re.I)!=None:
        return filename

def getalldocfilename(path):
    docfilenames = []
    sub_subjects = []
    for dirpath, dirnames, filenames in os.walk(path):
        # for dirname in dirnames: 
        # print(f"LINE: {dirpath[len(path)+1:]}, {filenames}")
        sub_subject = {}
        sub_subject[dirpath[len(path)+1:]] = filenames
        sub_subjects.append(sub_subject)

        for filename in filenames:
            # print(f"filename: {filenames}")
            filenames = filter(filterFileName, filenames)
            filenames = map(lambda filename: os.path.join(dirpath, filename), filenames)
            docfilenames.extend(filenames)

    return docfilenames, sub_subjects

def main():
    top_readme_filepath = "README-test.md"
    first_open = False 
    top_readme_title = "test"

    print ('begin')
    path= "/home/magic/work/gitwork/work_note/ubuntu_os_usage"
    files, sub_subjects = getalldocfilename(path)
    print(sub_subjects)

    for file in files:
        # print (file)
        blog_title = get_file_title(file)
        https_path = 'https://www.github.com/magic428/work_note/blob/master' + (file[file.find("work_note")+len("work_note"):]).replace(os.sep, '/')
        item = '- [{}]({})'.format(blog_title, https_path) + '  \n'

        if first_open:
            first_open = False
            with open(top_readme_filepath,'w', encoding='utf-8') as f:
                f.write("# " + top_readme_title + '\n')
                f.write('\n## Introduce\n\n')
                f.write(item)

        else:
            with open(top_readme_filepath,'a', encoding='utf-8') as f:
                f.write(item)


if __name__=='__main__':
    main()