/*
 * Copyright (c) 2018 TiEV (Tongji Intelligent Electric Vehicle).
 * 
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 * 
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */

  const char features[] = {"\n"
"C_FEATURE:"
#if (__GNUC__ * 100 + __GNUC_MINOR__) >= 404
"1"
#else
"0"
#endif
"c_function_prototypes\n"
"C_FEATURE:"
#if (__GNUC__ * 100 + __GNUC_MINOR__) >= 404 && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
"1"
#else
"0"
#endif
"c_restrict\n"
"C_FEATURE:"
#if (__GNUC__ * 100 + __GNUC_MINOR__) >= 406 && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201000L
"1"
#else
"0"
#endif
"c_static_assert\n"
"C_FEATURE:"
#if (__GNUC__ * 100 + __GNUC_MINOR__) >= 404 && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
"1"
#else
"0"
#endif
"c_variadic_macros\n"

};

int main(int argc, char** argv) { (void)argv; return features[argc]; }
