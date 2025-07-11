
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import os
import glob
from generate_audio_kokoro import generate_audio

app = FastAPI()

# Mount static files to serve CSS, JS and audio
app.mount("/static", StaticFiles(directory="static"), name="static")
app.mount("/audio", StaticFiles(directory="audio"), name="audio")

class TTSRequest(BaseModel):
    text: str
    voice: str

def get_available_voices():
    """Scans the directory for available voice models."""
    voice_folders = glob.glob("z[fm]_*")
    return [folder.replace('\\', '/').split('/')[-1] for folder in voice_folders]

@app.get("/", response_class=HTMLResponse)
async def read_root():
    """Serves the main HTML page."""
    with open("templates/index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

@app.get("/api/voices")
async def get_voices():
    """Endpoint to get the list of available voices."""
    return {"voices": get_available_voices()}

@app.post("/api/tts")
async def text_to_speech(request: TTSRequest):
    """Endpoint to generate audio from text."""
    if not request.text or not request.voice:
        raise HTTPException(status_code=400, detail="Text and voice must be provided.")

    try:
        # Ensure the audio directory exists
        if not os.path.exists("audio"):
            os.makedirs("audio")

        # Generate a unique filename based on voice and text hash
        # Using a hash of the text ensures that the same text generates the same filename
        output_filename = f"audio/{request.voice}_{hash(request.text)}.wav"
        
        # Call the refactored TTS generation function
        generate_audio(text=request.text, voice=request.voice, output_path=output_filename)

        # Return the path to the generated audio file for the frontend
        return {"audio_url": f"/{output_filename}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    # Ensure the required directories exist
    if not os.path.exists("templates"):
        os.makedirs("templates")
    if not os.path.exists("static"):
        os.makedirs("static")
    if not os.path.exists("audio"):
        os.makedirs("audio")
        
    print("Available voices:", get_available_voices())
    print("Starting server...")
    uvicorn.run(app, host="0.0.0.0", port=8000)
