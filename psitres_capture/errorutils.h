#include "stdafx.h"
#include <FlyCapture2.h>
#include <stdexcept>
#include <sstream>

#define __STRING(x) #x

#ifndef _DEBUG                  /* For RELEASE builds */
#define DBG(expr)  do {;} while (0)
#else                           /* For DEBUG builds   */
#define  DBG(expr) do { (expr); } while (0)
#endif

#define assert_throw(expr) ((expr) ? (static_cast<void>(0)) : throw AssertionError(__FILE__, __LINE__, __STRING(expr)))

#define PG_CheckError(expr) \
{ \
	Error error = (expr); \
	if (error != PGRERROR_OK) throw PGError(__FILE__, __LINE__, __STRING(expr), error); \
} \

#define PG_Call(expr) \
{ \
	DBG(cerr << __STRING(expr) << endl); \
	PG_CheckError(expr); \
} \

class PGError : public std::runtime_error {
public:
	PGError(const std::string& file, const int& line, const std::string& expr,
		const FlyCapture2::Error& error) :
		std::runtime_error(PGError::createMsg(file, line, expr, error)), error(
		error) {
	}
	~PGError() throw () {
	}
	static std::string createMsg(const std::string& file, const int& line,
		const std::string& expr, const FlyCapture2::Error& error) {
		std::stringstream ss;
		const char * errorStr = error.GetDescription();
		ss << file << "[" << line << "] " << expr << " PGError: " << errorStr
			<< "(" << error.GetType() << ")";
		return ss.str();
	}
	const FlyCapture2::Error error;
};

class AssertionError : public std::runtime_error {
public:
	AssertionError(const std::string& file, const int& line,
		const std::string& expr) :
		std::runtime_error(AssertionError::createMsg(file, line, expr)) {
	}
	std::string createMsg(const std::string& file, const int& line,
		const std::string& expr) {
		std::stringstream ss;
		ss << file << "[" << line << "] " << expr << " AssertionError: assertion failed";
		return ss.str();
	}
};