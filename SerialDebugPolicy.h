#pragma once

namespace pn532
{
struct SerialDebug
{
	static void begin(uint32_t baud) { Serial.begin(baud); }

	template <typename T, typename U>
	static size_t print(T t, U u) { return Serial.print(t, u); }

	template <typename T>
	static size_t print(T t) { return Serial.print(t); }

	template <typename T, typename U>
	static size_t println(T t, U u) { return Serial.println(t, u); }

	template <typename T>
	static size_t println(T t) { return Serial.println(t); }

	static size_t println() { return Serial.println(); }
};

}
