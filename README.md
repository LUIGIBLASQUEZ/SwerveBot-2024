# Code Documentation for Robotics 691 Robot
 
## Documentation Basics

For those who are unfamiliar with documentation, one of the key goals of a programmer is to ensure they leave proper documentation and records of their material. This includes comments on code, and proper repository/directory storage of the code you handle. The best way to think of documentation, is as a way to be able to share your code with your team, amongst members who would have little to no knowledge on what you were making/programming.  

By making it readable through proper documentation, any user who may be not updated on recent changes, should be able to come in and smoothly interpret and understand the changes and functions implemented. 

## Getting started

Most importantly, you maybe already understand documentation, or the language you are programming in, but to properly handle code on this site, you need to understand how to handle a GitHub repository. Here I will list some information on how to interact with it using the sites given commands. 

Let's look at this from a high level perspective. Code is stored and accessed in repositories. A repository is a central location in which code/data is stored and managed. GitHub allows for multiple users to upload (push), and download (pull) freely for the rest of the team to access. In these repositories, files are divided up into what is considered directories, directories are a file which consists solely of a set of other files (which may themselves be directories). 

In order to push a command, we have to access the repository using the 'cd' command, which stands for 'change directory', which sends you to the directory you need to be. For example, we execute
```
cd existing_directory/dir_file
```
In this command we change our current directory to locate at dir_file, within the existing_directory. 
From here we add the file to the current repository using:
```
git add <file>
```
git sets this as a git specific command, and then the add implements the file you label under <file> (no <> needed)

The file is now added, however you will need to assign a branch, or choose a branch. An example is: 
```
git branch -M main
```
This would assign or rename the "master" branch as a branch named main. Each repository comes with a "master" branch (essentially the first branch), often known as the main. 

In any other case where we are not assigning the first of a repository, simple put:
```
git branch <example_branch_name>
```
After the file is added, and the branch is assigned (if needed), we then can commit it to the directory. 
```
git commit -m "DESCRIBE COMMIT IN A FEW WORDS"
```
Commit essentially saves the file in the directory, and the -m, allows a message to be left, which is provided like the example "given comment".

Finally, by using push, we would be essentially making our changes to the file. 

```
In the case of a main branch, it is:
git push -u origin main

and in other branches, it is:

git push -u origin <branch-name> 
```
--------------------------------------------------------------------------------------------------------------

When you have to pull, do the following command: 
```
git pull https://github.com/LUIGIBLASQUEZ/robot-2024-2.git
```

(source: https://www.atlassian.com/git/tutorials/syncing/git-pull#:~:text=The%20git%20pull%20command%20is,Git%2Dbased%20collaboration%20work%20flows.)

# Add your files (extra documentation)

- [ ] [Create](https://docs.GitHub.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.GitHub.com/ee/user/project/repository/web_editor.html#upload-a-file) files

- [ ] [Add files using the command line](https://docs.GitHub.com/ee/GitHub-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://github.com/LUIGIBLASQUEZ/robot-2024-2.git
git branch -M main
git push -uf origin main
```


## Collaborate with your team
Listed below are some https (Hypertext Transfer Protocol Secure), to link you to GitHub. 
- [ ] [Invite team members and collaborators](https://docs.GitHub.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.GitHub.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.GitHub.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.GitHub.com/ee/user/project/merge_requests/approvals/)
- [ ] [Automatically merge when pipeline succeeds](https://docs.GitHub.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

<!-- ## Test and Deploy

Use the built-in continuous integration in GitHub.

- [ ] [Get started with GitHub CI/CD](https://docs.GitHub.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing(SAST)](https://docs.GitHub.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.GitHub.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.GitHub.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.GitHub.com/ee/ci/environments/protected_environments.html) -->

***

