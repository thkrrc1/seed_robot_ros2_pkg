#pragma once

#include "common.hpp"

/**
 * extensionは、'.'込みで指定する。
 */
std::vector<std::string> getFileNames(std::string folderPath, std::string extension, std::string prefix = "", bool recursive = false) ;
std::vector<std::string> searchDirectoryPath(std::string tgtPath, std::string name);
std::vector < std::string > getFileIncludeDirectoryList(std::string tgtPath,std::string fname);
bool makeDirectory(std::string dir);
bool removeDirectory(std::string dir);
bool moveDirectory(std::string dir_old,std::string dir_new);
void removeFile(std::string fpath);
void copyFile(std::string from_path,std::string to_path,bool recursive = false);
void renameFile(std::string from_path,std::string to_path);
bool fileExist(std::string fpath);
std::string getDataDirectory();
std::string getExtension(const std::string &fpath);

/**
 * ファイルパスから、ファイル名を抽出する。
 * @param stem_only 拡張子を含めない場合はtrue
 */
std::string getFileName(const std::string &path, bool stem_only = false);

std::string getParentPath(const std::string &path);
