#ifndef DRB_EXTERNAL_BREAKABLE_SCOPE_H
#define DRB_EXTERNAL_BREAKABLE_SCOPE_H

// MIT License
//
// Copyright (c) 2017 Clemens Sielaff
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/// Use `breakable_scope` to write:
///
///     breakable_scope {
///         ...
///         if(!condition){
///             break;
///         }
///         ...
///     }
///     else{ // called when the breakable_scope is exited via `break`
///         handle_error();
///     }
///
/// If you exit the scope via `continue`, it will break the scope without
/// executing the `else` section at the end.
///
#define breakable_scope                              \
  for (auto _breakable_scope_condition_variable = 0; \
       _breakable_scope_condition_variable < 2;      \
       _breakable_scope_condition_variable += 1)     \
    for (; _breakable_scope_condition_variable < 2;  \
         _breakable_scope_condition_variable += 2)   \
      if (_breakable_scope_condition_variable == 0)

#endif