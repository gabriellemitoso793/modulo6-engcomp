from flask import request, jsonify, render_template
from app import app
from models.model import load_model, preprocess_image
from PIL import Image
import io

model = load_model()

@app.route('/predict', methods=['POST'])
def predict():
    if 'file' not in request.files:
        return jsonify({'error': 'No file uploaded'}), 400
    
    file = request.files['file']
    image = Image.open(io.BytesIO(file.read()))
    processed_image = preprocess_image(image)
    
    prediction = model.predict(processed_image).argmax(axis=1)[0]
    return jsonify({'digit': int(prediction)})

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        if 'file' not in request.files:
            return 'No file uploaded', 400
        
        file = request.files['file']
        image = Image.open(io.BytesIO(file.read()))
        processed_image = preprocess_image(image)
        
        prediction = model.predict(processed_image).argmax(axis=1)[0]
        return f'The digit is: {prediction}'
    
    return render_template('index.html')


