#ifndef DRB_ASSERT_H
#define DRB_ASSERT_H

#define DRB_DEFAULT_ASSERT 1
#define DRB_LOG_ON_ASSERT 2
#define DRB_TERMINATE_ON_ASSERT 3

#if DRB_ASSERT_MODE == DRB_TERMINATE_ON_ASSERT
#include <iostream>
#include <exception>
#define ASSERT(exp, msg) if(!(exp)) { std::cerr << "ASSERTION @ [ " << __FILE__ << ": Line " << __LINE__ << "] Expr: " << #exp << ". Msg: " << msg << '\n'; std::terminate(); } void(0)

#elif DRB_ASSERT_MODE == DRB_LOG_ON_ASSERT
#include <iostream>
#define ASSERT(exp, msg) if(!(exp)) { std::cerr << "ASSERTION @ [ " << __FILE__ << ": Line " << __LINE__ << "] Expr: " << #exp << ". Msg: " << msg << '\n'; } void(0)

#elif DRB_ASSERT_MODE == DRB_DEFAULT_ASSERT
#ifdef _DEBUG
#include <cassert>
#define ASSERT(exp, msg) assert((exp) && (msg))
#else
#define ASSERT(exp, msg) void(0)
#endif

#else
#define ASSERT(exp, msg) void(0)
#endif

#endif

