# UW Robotics Team
## Robot Racing

### Description:
This repository contains the current competition code (2019), which will be used used for IARRC 2019. A list of what we're currently working on is in the issues section of this repo.

### Getting Started:
Here is a good doc that outlines everything you need to know in order to get set up and working with the codebase!
* https://docs.google.com/document/d/1ZDILnQNoG6NpTkf0U2AT2KQerttBemVC6pHEtwzu-uU/edit

### Competition Rules:
Please review the competition rules before pushing any code :)
* https://iarrc.org/rules/

### Coding Standards:
For C++ code and Python Scripts we follow the Google coding standards linked below
* https://google.github.io/styleguide/cppguide.html
* https://google.github.io/styleguide/pyguide.html

Git workflow that we follow
* https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow

Please fill out this header above all functions (Doxygen style)
``` cpp
/** @brief Writes the current position of the cursor
 *         into the arguments row and col.
 *  @param row The address to which the current cursor
 *         row will be written.
 *  @param col The address to which the current cursor
 *         column will be written.
 *  @return Void.
 */
void GetCursor(int* row, int* col);
```
Please fill this header out above every source code file(including launch files and xml files)
```
/** @file console.h
 *  @brief Function prototypes for the console driver.
 *
 *  This contains the prototypes for the console
 *  driver and eventually any macros, constants,
 *  or global variables you will need.
 *
 *  @author YOUR NAME (GITHUB_USERNAME)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
```
In order to generate documentation, go to the base directory of this repo and run:
``` bash
doxygen doxygen_config
```
It is setup to document all \*.c, \*.cpp, \*.h, and \*.hpp files. It will generate html output under doxygen\_output/html. To view this, reference the index.html file in that directory in a browser of your choice. Feel free to change the doxygen\_config file in order to generate different types of documentation or cover more file types.

### Contributers for the current code:
* Waleed Ahmed (Software Lead)
* Andrew Jin
* Jack Xu

### Past Contributers (2014-2017):
* Toni Ogunmade (Software Lead)
* Adrian Malaran
* Angela Gu
* Brian Kibazohi
* Matthew Post (Software Lead)
* Praveen Dorairaj
* Ivy Xing
* Aditya Matam
* Ee Ern Low
* Sirui Song
* Michael Smart
* Jungwook Lee (Software Lead)
* Raymond Kuo (Software Lead)
* Jakub Dworakowski (Software Lead)
* Greg Varty
* Archie Lee
* Shalin Upadhyay
* Ning Zhao
* Jason Leung
* Jamie Kim
