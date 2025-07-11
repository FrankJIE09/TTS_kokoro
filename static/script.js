document.addEventListener('DOMContentLoaded', () => {
    const voiceSelect = document.getElementById('voice-select');
    const generateBtn = document.getElementById('generate-btn');
    const textInput = document.getElementById('text-input');
    const audioOutput = document.getElementById('audio-output');
    const audioPlayer = document.getElementById('audio-player');
    const downloadLink = document.getElementById('download-link');
    const loadingSpinner = document.getElementById('loading-spinner');

    // Fetch available voices and populate the dropdown
    fetch('/api/voices')
        .then(response => response.json())
        .then(data => {
            voiceSelect.innerHTML = '<option value="">-- 选择一个声音 --</option>'; // Clear loading text
            data.voices.forEach(voice => {
                const option = document.createElement('option');
                option.value = voice;
                option.textContent = voice;
                voiceSelect.appendChild(option);
            });
        })
        .catch(error => {
            console.error('Error fetching voices:', error);
            voiceSelect.innerHTML = '<option value="">无法加载声音</option>';
        });

    // Handle the generate audio button click
    generateBtn.addEventListener('click', () => {
        const text = textInput.value.trim();
        const voice = voiceSelect.value;

        if (!text) {
            alert('请输入文本！');
            return;
        }
        if (!voice) {
            alert('请选择一个声音！');
            return;
        }

        // Show loading spinner and hide previous results
        loadingSpinner.classList.remove('hidden');
        audioOutput.classList.add('hidden');
        generateBtn.disabled = true;

        // Send the request to the backend
        fetch('/api/tts', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ text, voice }),
        })
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            // Update the audio player and download link
            audioPlayer.src = data.audio_url;
            downloadLink.href = data.audio_url;
            downloadLink.download = data.audio_url.split('/').pop();

            // Show the audio output section
            audioOutput.classList.remove('hidden');
        })
        .catch(error => {
            console.error('Error generating audio:', error);
            alert('音频生成失败，请查看控制台了解详情。');
        })
        .finally(() => {
            // Hide loading spinner
            loadingSpinner.classList.add('hidden');
            generateBtn.disabled = false;
        });
    });
});
