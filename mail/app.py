from flask import Flask, render_template, request
from flask_mail import Mail, Message
from werkzeug.utils import secure_filename
from doc_split import split_file

# create flask server
app = Flask(__name__)

# flask-mail configuration
app.config['DEBUG'] = False
app.config['TESTING'] = False
app.config['MAIL_SERVER'] = 'smtp.gmail.com'
app.config['MAIL_PORT'] = 587
app.config['MAIL_USE_TLS'] = True
app.config['MAIL_USE_SSL'] = False
app.config['MAIL_DEBUG'] = False
app.config['MAIL_USERNAME'] = 'dranseika.mantas@gmail.com'
app.config['MAIL_PASSWORD'] = 'poyggbeprycpjshw'
app.config['MAIL_DEFAULT_SENDER'] = 'dranseika.mantas@gmail.com' 
app.config['MAIL_MAX_EMAILS'] = 5
app.config['MAIL_SUPPRESS_SEND'] = False
app.config['MAIL_ASCII_ATTACHMENTS'] = False

# create mail server
mail = Mail(app)

# root route that contains upload form
@app.route('/')
def index():
  """Main route of the web application

  Returns:
    html: returns html form (index.html)
  """
  return render_template('index.html')


# upload route that uploads, splits and sends mail messages
@app.route('/send', methods = ['POST'])
def send():
  """post route of the web application

  Returns:
    html: returns html page (upload.html)
  """
  if request.method == 'POST':
      
    # save file
    f = request.files['file']
    f.save(secure_filename(f.filename))       
    
    # splits file into individual salary slips and saves employees data
    employees = split_file(f.filename)
    
    # send all emails to individual employees
    for employee in employees:            
      msg = Message('Salary slip', recipients=[employee[3]])
      msg.body = 'Hello. I\'m send you your salary slip. You don\'t have to reply.'
      with app.open_resource(employee[4]) as file:
        msg.attach(employee[4], 'application/vnd.openxmlformats-officedocument.wordprocessingml.document', file.read()) # attach file of type .docx to the email message
      mail.send(msg)

  
    return render_template('upload.html') # render upload message
  
    
if __name__ == '__main__':
  app.run()