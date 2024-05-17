import cv2
import numpy as np
import requests
from io import BytesIO


# 웹사이트에서 영상을 받아오는 함수
def get_video_from_website(url):
    response = requests.get(url)
    return BytesIO(response.content)

    # 사람을 인식하는 함수
def detect_people(video_url):
	# 영상 받아오기
	video_stream = get_video_from_website(video_url)


	# OpenCV 비디오 캡처 생성
	cap = cv2.VideoCapture(video_stream)

	# 사람 인식을 위한 Haar Cascade 분류기 불러오기
	cascade_path = cv2.data.haarcascades + 'haarcascade_fullbody.xml'
	body_cascade = cv2.CascadeClassifier(cascade_path)

	while True:
		ret, frame = cap.read()  # 비디오 프레임 읽기
		if not ret:
			break

			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
			bodies = body_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))  # 사람 감지

			# 감지된 사람 주위에 사각형 그리기
			for (x, y, w, h) in bodies:
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

		# 영상 화면에 표시
		cv2.imshow('Video', frame)

		# 'q'를 누르면 종료
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

    # 비디오 캡처 해제
    cap.release()
	cv2.destroyAllWindows()

# 웹사이트에서의 영상 URL
video_url = "http://example.com/video.mp4"

# 영상에서 사람 인식 실행
detect_people(video_url)
