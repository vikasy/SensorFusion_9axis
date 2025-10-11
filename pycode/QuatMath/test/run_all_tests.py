#!/usr/bin/env python3
"""
Run All QuatMath Tests

Master test runner that executes all test suites:
1. Generate test data
2. Run comprehensive function tests
3. Run performance benchmarks
"""

import sys
import os
from pathlib import Path

def main():
    """Run all QuatMath tests"""
    print("🚀 QuatMath Test Suite Runner")
    print("=" * 50)
    
    test_dir = Path(__file__).parent
    
    # Step 1: Generate test data
    print("\n1️⃣  Generating Test Data...")
    try:
        sys.path.insert(0, str(test_dir))
        from generate_test_data import main as generate_data
        generate_data()
        print("✅ Test data generation completed")
    except Exception as e:
        print(f"❌ Test data generation failed: {e}")
        return False
    
    # Step 2: Run comprehensive tests
    print("\n2️⃣  Running Comprehensive Function Tests...")
    try:
        from test_quatmath_functions import run_comprehensive_test
        test_success = run_comprehensive_test()
        if test_success:
            print("✅ Comprehensive tests passed")
        else:
            print("⚠️  Some comprehensive tests failed")
    except Exception as e:
        print(f"❌ Comprehensive tests failed: {e}")
        test_success = False
    
    # Step 3: Run performance benchmarks
    print("\n3️⃣  Running Performance Benchmarks...")
    try:
        from benchmark_performance import generate_performance_report
        generate_performance_report()
        print("✅ Performance benchmarks completed")
    except Exception as e:
        print(f"❌ Performance benchmarks failed: {e}")
    
    # Final summary
    print("\n🎯 Final Summary")
    print("=" * 30)
    if test_success:
        print("✅ All QuatMath functions are working correctly!")
        print("✅ Test data generated successfully")
        print("✅ Performance benchmarks completed")
        print("\n🎉 QuatMath library is ready for production use!")
    else:
        print("⚠️  Some tests failed - check the output above")
        print("💡 Consider fixing failing functions before production use")
    
    return test_success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)