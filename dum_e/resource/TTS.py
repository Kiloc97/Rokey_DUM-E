import openai
import os
import simpleaudio as sa  # pip install simpleaudio
from dotenv import load_dotenv

load_dotenv(dotenv_path=os.path.join(".env"))

# OpenAI API 키 설정
openai.api_key = os.getenv("OPENAI_API_KEY")  # 환경변수에서 불러오기 권장

# 또는 직접 키 입력 (주의: 노출되면 위험)
# openai.api_key = "sk-..."

def text_to_speech(text, voice="nova", output_file="output.wav"):
    try:
        # OpenAI TTS API 호출
        response = openai.audio.speech.create(
            model="tts-1",  # 또는 "tts-1-hd"
            voice=voice,    # nova, shimmer, echo 등 중 선택
            input=text,
        )

        # 오디오 저장
        with open(output_file, "wb") as f:
            f.write(response.content)

        print(f"✅ '{output_file}'로 저장되었습니다.")
        return output_file

    except Exception as e:
        print("❌ 오류 발생:", e)
        return None


def play_audio(file_path):
    # simpleaudio로 음성 재생
    try:
        wave_obj = sa.WaveObject.from_wave_file(file_path)
        play_obj = wave_obj.play()
        play_obj.wait_done()
        print("🎧 음성 재생 완료.")
    except Exception as e:
        print("❌ 음성 재생 오류:", e)


if __name__ == "__main__":
    user_input = input("🎤 음성으로 출력할 텍스트를 입력하세요: ")
    audio_path = text_to_speech(user_input)
    if audio_path:
        play_audio(audio_path)
