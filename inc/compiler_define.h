//
// Created by hujianzhe
//

#ifndef UTIL_C_COMPILER_DEFINE_H
#define	UTIL_C_COMPILER_DEFINE_H

#ifdef _MSC_VER
	#pragma warning(disable:4200)
	#pragma warning(disable:4018)
	#pragma warning(disable:4244)
	#pragma warning(disable:4267)
	#pragma warning(disable:4800)
	#pragma warning(disable:4819)
	#pragma warning(disable:4996)
	#pragma warning(disable:6255)
	#pragma warning(disable:26451)

	#define	__declspec_dllexport					__declspec(dllexport)
	#define	__declspec_dllimport					__declspec(dllimport)

	#ifdef	DECLSPEC_DLL_EXPORT
		#define	__declspec_dll						__declspec_dllexport
	#elif	DECLSPEC_DLL_IMPORT
		#define	__declspec_dll						__declspec_dllimport
	#else
		#define	__declspec_dll
	#endif

#elif	defined(__GNUC__) || defined(__GNUG__)
	#ifndef NDEBUG	/* ANSI define */
		#ifndef _DEBUG
			#define	_DEBUG	/* same as VC */
		#endif
	#else
		#undef	_DEBUG	/* same as VC */
	#endif

	#define	__declspec_dllexport					__attribute__((visibility("default")))
	#define	__declspec_dllimport

	#ifdef	DECLSPEC_DLL_EXPORT
		#define	__declspec_dll						__declspec_dllexport
	#elif	DECLSPEC_DLL_IMPORT
		#define	__declspec_dll						__declspec_dllimport
	#else
		#define	__declspec_dll
	#endif

	#ifdef	__clang__
		#if	__has_feature(address_sanitizer)
			#ifndef	__SANITIZE_ADDRESS__
				#define	__SANITIZE_ADDRESS__
			#endif
		#endif
	#endif

#else
	#error "Unknown Compiler"
#endif

#endif
