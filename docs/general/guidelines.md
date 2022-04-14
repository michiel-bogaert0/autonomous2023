# Branching strategy

![image](https://user-images.githubusercontent.com/78962099/163336816-8cb8d6b0-6ec2-4196-9718-b03c66630d81.png)

We will use a git flow based branching strategy, resulting in the following rules:

## Strategy

* We have one main branch that should be stable at all times.
  * The only way for updates to be pushed to the stable main are through hotfixes or releases. These should always be code reviewed before pushing.
* There is a permanent development branch which is used for developers to work on.
  * Development happens using feature branches from the development branch. Bugfixes to the development branch are done in feature branches with a specific name, see next section.
  * The development branch can be merged into the main branch using a release branch to make sure the features are robust prior to merging with the main.
* Bugfixes are done in a feature branch, but named differently, see next section.

## Naming

All branch names start with tag to indicate type followed by descriptive name.

* Bugfix = B (small fix to the development branch that can be finished in 1-2 days)
* Feature = F (a project that can be finished in a relatively short time, usually 1-2 people)
* Hotfix = H (small fix to the main branch that can be finished within a day)
* Release = R (release branch to transfer features from the development branch into the main branch for deployment and product release)

Examples:

* F_auto_download_data
* B_bgr_rgb_fix
* H_fix_launch_file
* R_0.1

## Workflow

* Create feature branch from the development branch
* Implement the desired feature
* Assure the code is linted, type checked and adequately tested
* Merge with the development branch

Before a release
* Create a release branch
* Make sure the branch is stable and passes all tests
* Work out the final bugs and create a merge with the main branch
* Assign the code review to someone
* Implement code review remarks
* Check the CI/CD pipeline (if implemented)
* Merge the code

## Remarks

* Feature branches should be limited to one feature.
* Try to limit the amount of work done in one feature branch, this makes code reviews easier and improved the clarity.

# Code style

## Python
Our coding style is defined by a linter, which in our case is [black](https://github.com/psf/black). Black makes sure that indentation, line length, vertical spacing etc is consistent over all the files. The functionality of black is further enhanced by isort and autoflake. [Isort](https://github.com/PyCQA/isort) sort the imports such that they are grouped by type and sorted alphabetically. [Autoflake](https://pypi.org/project/autoflake/) on the other hand removes unused imports. **Files that do not pass the linter check will not be committed to the master branch.** Linting can be done by using the lint script by running `scrips/lint.sh` inside the autonomous folder, make sure that the correct linter versions are installed.

Function comments should be used, specifically using the [Google style](https://google.github.io/styleguide/pyguide.html), most IDE's allow you to configure the comment style for you so just check with your specific one.

Further remarks about style:
* Keep function lengths reasonable (<80 lines), split up into multiple functions to improve readability.
* Avoid nesting more than two loops, this can often be avoided by using functions.
* Don't comment out code.
* Don't optimize on line length, if the code is clearer with more lines then use more lines. This also means no excessive usage of list/dict comprehension since this often makes code harder to understand.
* All variable names should be sensible and explain their usage. This means that single characters variables are discouraged even in simple loops.
* Constants are preferably enclosed in an Enum class.
* f-string is preferred over the % operator.
* We follow the python naming convention:
  *  Classes => CamelCase
  *  Functions and variables => snake_case

## C/C++
We use the Clang linter, to set it up, install the Ubuntu package using: `sudo apt install clang-format-10`.
To lint an entire directory, including sub-directories: `find ~/autonomous/hardware -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-10 -i -style=file`

# Code content

* No wildcard imports.
* All code should be typed and checked with [mypy](https://github.com/python/mypy). Typing can be ignored in external libaries, but if code is copied to our repositories it should be typed.
* Only catch the expected exception with a try/catch block. This avoids hiding errors.
* Don't use dictionaries to replace classes. If the form is fixed (e.g. configs) then use a class.
* Don't use global variables.
* Throw an exception if you are in a state you can't handle. Better to provide a clear error than an unclear one down the line.
* Use logging instead of the print function. Print function should only be used if the user has to see the output such as in a CLI.
* Functions should be tested as much as possible, functions that are not testable should be split up as much as possible to minimize the untested code.

# ROS

* Remove all template comments after package creation (All lines starting with #)
* If the package has specific requirements place those in a requirements.txt file at the root of the package. Example: /ROS/src/node_fixture/requirements.txt
