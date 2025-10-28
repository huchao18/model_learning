<title></title><style>
@font-face{
font-family:"Times New Roman";
}
@font-face{
font-family:"宋体";
}
@font-face{
font-family:"Wingdings";
}
@font-face{
font-family:"黑体";
}
@list l0:level1{
mso-level-number-format:decimal-enclosed-circle-chinese;
mso-level-suffix:none;
mso-level-text:"%1　";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:0.0000pt;text-indent:20.0000pt;margin-left:0.0000pt;
font-family:宋体;}
@list l1:level1{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:21.2500pt;text-indent:-21.2500pt;font-family:'Times New Roman';}
@list l1:level2{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:28.3500pt;text-indent:-28.3500pt;font-family:'Times New Roman';}
@list l1:level3{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:35.4500pt;text-indent:-35.4500pt;font-family:'Times New Roman';}
@list l1:level4{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.%4.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:42.5000pt;text-indent:-42.5000pt;font-family:'Times New Roman';}
@list l1:level5{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.%4.%5.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:49.5500pt;text-indent:-49.5500pt;font-family:'Times New Roman';}
@list l1:level6{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.%4.%5.%6.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:56.7000pt;text-indent:-56.7000pt;font-family:'Times New Roman';}
@list l1:level7{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.%4.%5.%6.%7.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:63.7500pt;text-indent:-63.7500pt;font-family:'Times New Roman';}
@list l1:level8{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.%4.%5.%6.%7.%8.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:70.9000pt;text-indent:-70.9000pt;font-family:'Times New Roman';}
@list l1:level9{
mso-level-number-format:decimal;
mso-level-suffix:tab;
mso-level-text:"%1.%2.%3.%4.%5.%6.%7.%8.%9.";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:77.9000pt;text-indent:-77.9000pt;font-family:'Times New Roman';}
@list l2:level1{
mso-level-number-format:bullet;
mso-level-suffix:tab;
mso-level-text:"";
mso-level-tab-stop:none;
mso-level-number-position:left;
margin-left:21.0000pt;text-indent:-21.0000pt;font-family:Wingdings;}
p.MsoNormal{
mso-style-name:正文;
mso-style-parent:"";
margin:0pt;
margin-bottom:.0001pt;
word-break:break-all;
punctuation-wrap:simple;
mso-pagination:none;
text-align:justify;
text-justify:inter-ideograph;
font-family:'Times New Roman';
mso-fareast-font-family:宋体;
font-size:12.0000pt;
mso-font-kerning:1.0000pt;
}
h1{
mso-style-name:"标题 1";
mso-style-next:正文;
margin-top:17.0000pt;
margin-bottom:16.5000pt;
mso-para-margin-top:0.0000gd;
mso-para-margin-bottom:0.0000gd;
word-break:break-all;
punctuation-wrap:simple;
page-break-after:avoid;
mso-pagination:lines-together;
text-align:justify;
text-justify:inter-ideograph;
mso-outline-level:1;
line-height:240%;
font-family:'Times New Roman';
mso-fareast-font-family:黑体;
mso-ansi-font-weight:bold;
font-size:16.0000pt;
mso-font-kerning:22.0000pt;
}
span.10{
font-family:'Times New Roman';
}
span.15{
font-family:'Times New Roman';
color:rgb(128,0,128);
text-decoration:underline;
text-underline:single;
}
span.msoIns{
mso-style-type:export-only;
mso-style-name:"";
text-decoration:underline;
text-underline:single;
color:blue;
}
span.msoDel{
mso-style-type:export-only;
mso-style-name:"";
text-decoration:line-through;
color:red;
}
@page{mso-page-border-surround-header:no;
 mso-page-border-surround-footer:no;}@page Section0{
}
div.Section0{page:Section0;}</style>

# 1. **Git和Github的绑定**

参考文章：[Github入门教程，适合新手学习（非常详细）-CSDN博客](https://blog.csdn.net/black_sneak/article/details/139600633)

如果绑定过ssh，那么在用户下会有.ssh文件夹。所以输入<u><span><font face="Times New Roman">cd ~/.ssh</font></span></u>指令，如果未创建过就返回 "no such file or directory" 表明电脑没有ssh key，需要创建ssh key。

在终端输入 <u><span>ssh-keygen -t rsa -C “git<font face="宋体">账号</font><font face="Times New Roman">”</font></span></u>（账号是邮箱）。

连续进行 3 次回车Enter（确认）。

按路径进入 .ssh，里面存储的是两个ssh key 的秘钥，id_rsa.pub 文件里面存储的是公钥，id_rsa 文件里存储的是私钥，不能告诉别人。打开 id_rsa.pub 文件，复制里面的内容。登录github里面setting添加新的ssh_key。

添加成功后回到bash输入：ssh -T git@github.com——输入yes成功后配置信息。

git config --global user.name “gitname”

git config --global user.email “git邮箱”

到此绑定成功。

# 2. **新建仓库与github关联**

①　在github上面新建仓库

②　git终端 cd “本地路径”

③　git init  # 初始化本地 Git 仓库

④　将远程 GitHub 仓库添加到本地仓库（URL链接）：git remote add origin https://github.com/your-username/your-repository.git

⑤　推送过程：

l 暂存指令：git add 单个文件名（加引号）/.（目录下所有文件）

l 提交指令：git commit -m“失败信息”

l 提交指令：git push origin(远程仓库定义名) main(分支)

# 3. **附加指令**

测试ssh连接：ssh -T git@github.com

克隆仓库到本地：git clone 链接

**初始化仓库：git init     cd** **”****路径****”****→ →删除仓库：rm -rf .git**

仓库状态：git status

关联远程仓库：git remote add origin git@github.com:huchao18/model_learning.git

查询远程仓库信息： git remote -v

拉取仓库：git pull origin main

删除文件：https://liaoxuefeng.com/books/git/time-travel/delete/index.html
