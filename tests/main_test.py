import sys
import unittest

from robot_sphere_class_test import RobotSphereClassTest

if __name__ == "__main__":
    test_loader = unittest.TestLoader()
    test_runner = unittest.TextTestRunner()
    test_results = list()
    has_failed = False

    test_classes = [
        RobotSphereClassTest]

    for test_class in test_classes:
        test_results.append(
            test_runner.run(test_loader.loadTestsFromTestCase(test_class)))

    print("------------------------------------------------------------------")
    for i in range(len(test_results)):
        test_count = test_results[i].testsRun
        test_failures = test_results[i].failures
        test_errors = test_results[i].errors
        test_passed = test_count - len(test_failures)

        if len(test_failures) != 0:
            has_failed = True

        print(test_classes[i].__name__ + "[" + str(test_count) + " tests]:")
        print(str(test_passed) + " [\033[1m\033[92mpassed\033[0m]")
        print(str(len(test_failures)) + " [\033[1m\033[91mfailed\033[0m]")

    sys.exit(has_failed)
