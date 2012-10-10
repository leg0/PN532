#pragma once

namespace pn532
{

struct NopDebug
{
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

typedef NopDebug DebugPolicy;
} // pn532