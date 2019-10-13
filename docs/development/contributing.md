# Contributing Guide
This guide describes how to get a local copy of the repository, make a change, and 
contribute this change back to the lab.

Note this guide does not explore how to build or test changes to code (which should be done
prior to submitting your changes). For this information, view the [Developement Guide](development_guide).

## Join the uf-mil github organization
Our current workflow does not use forks, meaning in order to contribute you
need write access to the upstream repository. To do this you need
to be a member of the [uf-mil Github organization](https://github.com/orgs/uf-mil/people)
and part of the [develop group](https://github.com/orgs/uf-mil/teams/developers/members).



## Install git
To contribute changes, you will need to have a git client installed. This program will be used to track and upload your changes.


**Ubuntu**

`sudo apt install git`

**Windows**

For using git in windows, we suggest you use the [Github Desktop Client](https://desktop.github.com/)

## Cloning the repository

You need to clone (download) a copy of the repository onto your computer so you can make changes.

**Ubuntu**

First clone the upstream (MIL's fork) version of the repo:

`git clone --recursive https://github.com/uf-mil/mil.git`

**Windows**
Use the [Github Desktop GUI](https://help.github.com/en/desktop/contributing-to-projects/cloning-a-repository-from-github-to-github-desktop)

## Make and test your changes!
Now its time to contribute! You may find these guides helpful in making and testing changes:

* [Development guide](/docs/development/development_guide)
* [Adding Documentation](/docs/development/adding_documentation)

Be sure to only change the files you intend to and don't add any files that aren't needed.


## Commiting changes
Now that you have some super awesome, well tested, well documented, and clean changes, it's time to submit them for review!

You will need to provide a short description of your change when you commit it.


**Ubuntu**

* Create a new branch for your changes `git branch -b <branchname>`. Be sure to pick a descriptive name and make sure there is not [already a branch with that name](https://github.com/uf-mil/mil/branches)
* Look at the changes you made with `git status` and `git diff`
* Add the files you wish  to keep to staging `git add <file> <file...>` (or all files with `git add .`
* Commit the changes with a descriptive message `git commit -m "<message>"`
* Push your development branch to Github `git push origin <branchname>`

**Windows**
Follow [Github's guide for the Windows client](
https://help.github.com/en/desktop/contributing-to-projects/committing-and-reviewing-changes-to-your-project)

## Creating a pull request
Now that your changes have been commited, it's time to submit them to be reviewed and then merged into the repository on Github.

Follow [Github's guide for creating pull requests](https://help.github.com/en/desktop/contributing-to-projects/creating-a-pull-request)
