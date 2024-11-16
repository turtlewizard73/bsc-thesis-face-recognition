import timeit

# Note: This example is only tested with Python 3 (not Python 2)

# This is a very simple benchmark to give you an idea of how fast each step of face recognition will run on your system.
# Notice that face detection gets very slow at large image sizes. So you might consider running face detection on a
# scaled down version of your image and then running face encodings on the the full size image.
import os
PKG_DIR = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
print(str(PKG_DIR))

TEST_IMAGES = [
    f"{PKG_DIR}/test/test_images/quality/obama-240p.jpg",
    f"{PKG_DIR}/test/test_images/quality/obama-480p.jpg",
    f"{PKG_DIR}/test/test_images/quality/obama-720p.jpg",
    f"{PKG_DIR}/test/test_images/quality/obama-1080p.jpg"
]

output_filenames = [
    f"{PKG_DIR}/test/test_images/quality/output/o-obama-240p.jpg",
    f"{PKG_DIR}/test/test_images/quality/output/o-obama-480p.jpg",
    f"{PKG_DIR}/test/test_images/quality/output/o-obama-720p.jpg",
    f"{PKG_DIR}/test/test_images/quality/output/o-obama-1080p.jpg"
]

import face_recognition
known_image = face_recognition.load_image_file(f"{PKG_DIR}/test/test_images/quality/obama-1080p.jpg")
known_encoding = face_recognition.face_encodings(known_image)[0]

def run_test(setup, test, iterations_per_test=5, tests_to_run=10):
    fastest_execution = min(timeit.Timer(
        test,
        setup=setup,
        globals={
            'known_encoding': known_encoding,
            'output_filenames': output_filenames
        }
    ).repeat(tests_to_run, iterations_per_test))
    execution_time = fastest_execution / iterations_per_test
    fps = 1.0 / execution_time
    return execution_time, fps


setup_locate_faces = """
import face_recognition
image = face_recognition.load_image_file("{}")
"""

test_locate_faces = """
face_locations = face_recognition.face_locations(image)
"""

setup_face_landmarks = """
import face_recognition
image = face_recognition.load_image_file("{}")
face_locations = face_recognition.face_locations(image)
"""

test_face_landmarks = """
landmarks = face_recognition.face_landmarks(image, face_locations=face_locations)[0]
"""

setup_encode_face = """
import face_recognition
image = face_recognition.load_image_file("{}")
face_locations = face_recognition.face_locations(image)
"""

test_encode_face = """
encoding = face_recognition.face_encodings(image, known_face_locations=face_locations)[0]
"""

setup_end_to_end = """
import face_recognition
image = face_recognition.load_image_file("{}")
"""

test_end_to_end = """
encoding = face_recognition.face_encodings(image)[0]
"""

setup_recognition = """
import face_recognition
image = face_recognition.load_image_file("{}")
encoding = face_recognition.face_encodings(image)[0]
"""

test_recognition = """
results = face_recognition.compare_faces([known_encoding], encoding)
"""

setup_drawing = """
import face_recognition
import cv2
import os
image = face_recognition.load_image_file("{}")
output_file = "{}"
face_locations = face_recognition.face_locations(image)
encoding = face_recognition.face_encodings(image, known_face_locations=face_locations)[0]
results = face_recognition.compare_faces([known_encoding], encoding)
"""

test_drawing = """
top, right, bottom, left = face_locations[0]
cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
cv2.rectangle(image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
font = cv2.FONT_HERSHEY_DUPLEX
cv2.putText(image, str(results), (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
status = cv2.imwrite(output_file, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
"""

print("Benchmarks (Note: All benchmarks are only using a single CPU core)")
print()

for image, output_file in zip(TEST_IMAGES, output_filenames):
    size = image.split("-")[1].split(".")[0]
    print("Timings at resolution {}:".format(size))

    print(" - Face locations: {:.4f}s ({:.2f} fps)".format(*run_test(setup_locate_faces.format(image), test_locate_faces)))
    print(" - Face landmarks: {:.4f}s ({:.2f} fps)".format(*run_test(setup_face_landmarks.format(image), test_face_landmarks)))
    print(" - Encode face (from landmarks): {:.4f}s ({:.2f} fps)".format(*run_test(setup_encode_face.format(image), test_encode_face)))
    print(" - Encode face (from image): {:.4f}s ({:.2f} fps)".format(*run_test(setup_end_to_end.format(image), test_end_to_end)))
    print(" - Check for results: {:.4f}s ({:.2f} fps)".format(*run_test(setup_recognition.format(image), test_recognition)))
    print(" - Export results: {:.4f}s ({:.2f} fps)".format(*run_test(setup_drawing.format(image, output_file), test_drawing)))
    print()