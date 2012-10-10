#pragma once

namespace pn532
{

struct NopDebug
{
	static void begin() { }
	
	template <typename T>
	static void begin(T) { }

	template <typename T, typename U>
	static void print(T, U) { }

	template <typename T>
	static void print(T) { }

	template <typename T, typename U>
	static void println(T, U) { }

	template <typename T>
	static void println(T) { }

	static void println() { }
};

} // pn532