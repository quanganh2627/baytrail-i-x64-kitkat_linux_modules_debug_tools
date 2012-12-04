/*****************************************************************************
#                      INTEL CONFIDENTIAL
#
# Copyright 2009-2012 Intel Corporation All Rights Reserved.
#
# This file is provided for internal use only. You are free to modify it
# for your own internal purposes, but you cannot release it to the external
# open source community under any circumstances without SSG.DPD's approval.
#
# If you make significant improvements, fix bugs, or have suggestions for
# additional features or improvements, please provide your changes to
# Robert or Gautam for inclusion in this tool.
#
# Please contact Robert Knight (robert.knight@intel.com) or Gautam Upadhyaya
# (gautam.upadhyaya@intel.com) if you have any questions.
#
*****************************************************************************
*/

/*
 * File declaring some utility functions.
 */

#ifndef _PW_UTILS_HPP_
#define _PW_UTILS_HPP_

#include <string>
#include <vector>
#include <deque>


/*
 * A simple line reader.
 */
class LineReader{
private:
    LineReader();
    ~LineReader();
public:
    static void get_all_lines(const char *, std::vector<std::string>&);
    static void get_all_lines(const char *, std::deque<std::string>&);

    static void get_all_lines(FILE *, std::vector<std::string>&);
    static void get_all_lines(FILE *, std::deque<std::string>&);

    static int getline(FILE *, std::string&);
    static void trim(std::string&);
};


/*
 * A simple string tokenizer.
 */
class Tokenizer{
    typedef std::vector <std::string> str_vec_t;
public:
    Tokenizer(const std::string& str, const char *delim=NULL);
    const char *get_next_token();
    const char *get_next_token(const char *);

    const str_vec_t& get_all_tokens() const;
    const str_vec_t get_all_tokens(const char *);

private:
    size_t m_curr, m_next;
    std::string m_buffer;
    const char *m_delims;
    str_vec_t m_vec;
};

void extract_dir_and_file_from_path(const std::string& path, std::string& dir, std::string& file);
#endif // _PW_UTILS_HPP_
