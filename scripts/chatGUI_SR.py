#!/usr/bin/env python

import tkinter as tk
from tkinter import ttk, messagebox
from std_msgs.msg import String
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as PILImage, ImageTk
import io
import speech_recognition as sr
import pyttsx3
from gtts import gTTS
import tempfile
import os
import threading
import time  
import traceback 
import json
from datetime import datetime

class SILFeedbackPanel: 
    def __init__(self, parent, sil_feedback_publisher):
        self.parent = parent
        self.sil_feedback_publisher = sil_feedback_publisher
        self.current_interaction_id = None
        
        self.feedback_frame = tk.Frame(parent, bg="lightgray", bd=2, relief=tk.RAISED)
        self.feedback_frame.pack(fill=tk.X, padx=5, pady=2)

        feedback_title = tk.Label(self.feedback_frame, text="SIL Feedback & Status", 
                                font=("Arial", 10, "bold"), bg="lightgray")
        feedback_title.pack()

        satisfaction_frame = tk.Frame(self.feedback_frame, bg="lightgray")
        satisfaction_frame.pack(fill=tk.X, padx=5, pady=2)
        
        tk.Label(satisfaction_frame, text="Satisfaction:", bg="lightgray").pack(side=tk.LEFT)
        self.satisfaction_var = tk.IntVar(value=3)
        ratings = ["Poor", "Fair", "Good", "Great", "Excellent"]
        for i, text in enumerate(ratings, 1):
            tk.Radiobutton(satisfaction_frame, text=f"{i}:{text}", variable=self.satisfaction_var, 
                          value=i, bg="lightgray", font=("Arial", 8)).pack(side=tk.LEFT, padx=2)

        checkbox_frame = tk.Frame(self.feedback_frame, bg="lightgray")
        checkbox_frame.pack(fill=tk.X, padx=5, pady=2)
        
        self.feedback_vars = {
            'suggestions_helpful': tk.BooleanVar(),
            'needed_more_clarification': tk.BooleanVar(),
            'too_many_questions': tk.BooleanVar(),
            'response_too_slow': tk.BooleanVar(),
            'suggestions_annoying': tk.BooleanVar()
        }
        
        feedback_texts = {
            'suggestions_helpful': "Suggestions were helpful",
            'needed_more_clarification': "Needed more clarification",
            'too_many_questions': "Too many questions",
            'response_too_slow': "Response was too slow",
            'suggestions_annoying': "Suggestions were annoying"
        }
        
        for key, var in self.feedback_vars.items():
            tk.Checkbutton(checkbox_frame, text=feedback_texts[key], variable=var,
                          bg="lightgray", font=("Arial", 8)).pack(anchor='w')
            
        button_frame = tk.Frame(self.feedback_frame, bg="lightgray")
        button_frame.pack(fill=tk.X, padx=5, pady=2)
        
        tk.Button(button_frame, text="Send Feedback", command=self.send_feedback,
                 bg="#4CAF50", fg="white", font=("Arial", 9)).pack(side=tk.LEFT, padx=2)
        
        tk.Button(button_frame, text="Clear", command=self.clear_feedback,
                 bg="#f44336", fg="white", font=("Arial", 9)).pack(side=tk.LEFT, padx=2)
        
        self.status_text = tk.Text(self.feedback_frame, height=4, width=60, 
                                  font=("Arial", 8), bg="white", state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=2)
        
        command_frame = tk.Frame(self.feedback_frame, bg="lightgray")
        command_frame.pack(fill=tk.X, padx=5, pady=2)
        
        tk.Label(command_frame, text="SIL Command:", bg="lightgray", 
                font=("Arial", 8)).pack(side=tk.LEFT)
        self.command_entry = tk.Entry(command_frame, font=("Arial", 8))
        self.command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.command_entry.bind("<Return>", self.send_sil_command)
        
        tk.Button(command_frame, text="Send", command=self.send_sil_command,
                 bg="#2196F3", fg="white", font=("Arial", 8)).pack(side=tk.RIGHT)
    
    def send_feedback(self):
        try:
            feedback_data = {
                'timestamp': datetime.now().isoformat(),
                'satisfaction': self.satisfaction_var.get() / 5.0,  
                'interaction_id': self.current_interaction_id
            }
            
            for key, var in self.feedback_vars.items():
                feedback_data[key] = var.get()
            
            feedback_json = json.dumps(feedback_data)
            self.sil_feedback_publisher.publish(String(data=feedback_json))
            
            self.update_status(f"Feedback sent: Satisfaction {feedback_data['satisfaction']:.1f}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send feedback: {e}")
    
    def clear_feedback(self):
        self.satisfaction_var.set(3)
        for var in self.feedback_vars.values():
            var.set(False)
        self.update_status("Feedback cleared")
    
    def send_sil_command(self, event=None):
        command = self.command_entry.get().strip()
        if command:
            if hasattr(self.parent, 'sil_command_publisher'):
                self.parent.sil_command_publisher.publish(String(data=command))
                self.update_status(f"Command sent: {command}")
                self.command_entry.delete(0, tk.END)
    
    def update_status(self, message):
        self.status_text.config(state=tk.NORMAL)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
    
    def set_interaction_id(self, interaction_id):
        self.current_interaction_id = interaction_id
        self.update_status(f"New interaction: {interaction_id}")

class TTSInterface:
    def __init__(self, tts_topic=None):
        if tts_topic is None:
            tts_topic = rospy.get_param("topics/tts_text", "/tts_text")
        self.tts_subscriber = rospy.Subscriber(tts_topic, String, self.handle_tts)
        self.is_speaking = False
        self.engine = pyttsx3.init()
        self.sil_aware = True

        self.tts_engine_name = rospy.get_param("speech/tts_engine", "gtts")
        self.tts_language = rospy.get_param("speech/tts_language", "en")
        self.rate_normal = int(rospy.get_param("speech/tts_rate_normal", 180))
        self.rate_clarification = int(rospy.get_param("speech/tts_rate_clarification", 150))
        self.audio_player = rospy.get_param("speech/audio_player", "mpg123 -q")

        self.sil_tts_subscriber = rospy.Subscriber(
            rospy.get_param("topics/sil_tts", "/sil_tts"), String, self.handle_sil_tts)

    def handle_tts(self, msg):
        text = msg.data.strip()
        if text:
            self._speak(text, is_sil=False)
    
    def handle_sil_tts(self, msg):
        text = msg.data.strip()
        if text:
            self._speak(text, is_sil=True)
    
    def _speak(self, text, is_sil=False):
        self.is_speaking = True
        try:
            if is_sil and "clarification" in text.lower():
                self.engine.setProperty('rate', self.rate_clarification)
            else:
                self.engine.setProperty('rate', self.rate_normal)

            use_gtts = str(self.tts_engine_name).lower() == "gtts"
            if use_gtts:
                try:
                    tts = gTTS(text=text, lang=self.tts_language)
                    with tempfile.NamedTemporaryFile(suffix='.mp3', delete=True) as fp:
                        tts.save(fp.name)
                        os.system(f"{self.audio_player} {fp.name}")
                except Exception as e:
                    rospy.logerr(f"Google TTS error: {e}")
                    self.engine.say(text)
                    self.engine.runAndWait()
            else:
                self.engine.say(text)
                self.engine.runAndWait()
        finally:
            self.is_speaking = False

class EnhancedChatInterface(tk.Tk):
    def __init__(self):
        super().__init__()
        rospy.init_node('SIL_ChatGUI', anonymous=True)
        
        self.llm_input_publisher = rospy.Publisher(
            rospy.get_param("topics/llm_input", "/llm_input"), String, queue_size=10)
        self.sil_feedback_publisher = rospy.Publisher(
            rospy.get_param("topics/sil_feedback", "/sil_feedback"), String, queue_size=10)
        self.sil_command_publisher = rospy.Publisher(
            rospy.get_param("topics/sil_command", "/sil_command"), String, queue_size=10)
        
        rospy.Subscriber(rospy.get_param("topics/llm_output", "/llm_output"), String, self.add_llm_response)
        rospy.Subscriber(rospy.get_param("topics/llm_image_output", "/llm_image_output"), Image, self.add_llm_image)
        rospy.Subscriber(rospy.get_param("topics/sil_response", "/sil_response"), String, self.add_sil_response)
        rospy.Subscriber(rospy.get_param("topics/sil_clarification", "/sil_clarification"), String, self.add_clarification)
        rospy.Subscriber(rospy.get_param("topics/sil_suggestions", "/sil_suggestions"), String, self.add_suggestions)
        rospy.Subscriber(rospy.get_param("topics/sil_status", "/sil_status"), String, self.update_sil_status)
        
        self.bridge = CvBridge()
        self.setup_ui()
        self.setup_speech_recognition()

        self.tts_interface = TTSInterface()
        
        self.sil_enabled = True
        self.current_interaction_id = None
        self.interaction_history = []
        
        rospy.loginfo("[SIL-GUI] Chat Interface initialized")
    
    def setup_ui(self):
        self.title("SIL - Interact With Robot in Natural Language")
        self.geometry("700x800")
        self.configure(bg='white')
        
        self.chat_history = tk.Text(self, wrap=tk.WORD, state=tk.DISABLED, bg="#f0f0f0",
                                    padx=10, pady=10, font=("Arial", 11))
        self.chat_history.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.chat_history.tag_configure("sender", font=("Arial", 10, "bold"))
        self.chat_history.tag_configure("user_msg", foreground="green", background="#e6ffe6",
                                        lmargin1=10, lmargin2=10)
        self.chat_history.tag_configure("bot_msg", foreground="blue", background="#e6f0ff",
                                        lmargin1=10, lmargin2=10)
        self.chat_history.tag_configure("clarification", foreground="orange", background="#fff3e0",
                                        lmargin1=10, lmargin2=10)
        self.chat_history.tag_configure("suggestion", foreground="purple", background="#f3e5f5",
                                        lmargin1=10, lmargin2=10)
        self.chat_history.tag_configure("system", foreground="gray", background="#f5f5f5",
                                        lmargin1=10, lmargin2=10)
        
        self.input_frame = tk.Frame(self, bg="white")
        self.input_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.user_input_entry = tk.Entry(self.input_frame, bd=1, bg="white", font=("Arial", 12))
        self.user_input_entry.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        self.user_input_entry.bind("<Return>", self.send_message)
        self.user_input_entry.bind("<Up>", self.recall_previous_message)

        button_frame = tk.Frame(self.input_frame, bg="white")
        button_frame.pack(side=tk.RIGHT)
        
        self.send_button = tk.Button(button_frame, text="Send", command=self.send_message, bd=1,
                                     bg="#4CAF50", fg="white", font=("Arial", 11, "bold"), 
                                     activebackground="#45a049")
        self.send_button.pack(side=tk.LEFT, padx=2)
        
        self.listen_button = tk.Button(button_frame, text="Listen", command=self.start_listening_thread, bd=1,
                                       bg="#2196F3", fg="white", font=("Arial", 11, "bold"), 
                                       activebackground="#1E88E5")
        self.listen_button.pack(side=tk.LEFT, padx=2)
        
        self.sil_toggle_button = tk.Button(button_frame, text="SIL ON", command=self.toggle_sil, bd=1,
                                          bg="#FF9800", fg="white", font=("Arial", 10, "bold"))
        self.sil_toggle_button.pack(side=tk.LEFT, padx=2)
        
        self.clear_button = tk.Button(button_frame, text="Clear", command=self.clear_chat, bd=1,
                                     bg="#f44336", fg="white", font=("Arial", 10))
        self.clear_button.pack(side=tk.LEFT, padx=2)

        self.sil_panel = SILFeedbackPanel(self, self.sil_feedback_publisher)

        self.load_icons()

        self.message_history = []
        self.history_index = -1

        self.image_refs = []
    
    def load_icons(self):
        icons_dir = rospy.get_param("gui/icons_directory", "")
        if not icons_dir:
            try:
                import rospkg
                icons_dir = os.path.join(rospkg.RosPack().get_path('sil_ros'), 'icons')
            except Exception:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                icons_dir = os.path.join(script_dir, "..", "icons")
        try:
            you_img = PILImage.open(os.path.join(icons_dir, "you_icon.png"))
            llm_img = PILImage.open(os.path.join(icons_dir, "llm_icon.png"))
            you_img = you_img.resize((32, 32), PILImage.LANCZOS)
            llm_img = llm_img.resize((32, 32), PILImage.LANCZOS)
            self.you_icon = ImageTk.PhotoImage(you_img)
            self.llm_icon = ImageTk.PhotoImage(llm_img)
        except Exception as e:
            rospy.logwarn(f"Could not load icons from {icons_dir} ({e}); using placeholders")
            self.you_icon = tk.PhotoImage(width=16, height=16)
            self.llm_icon = tk.PhotoImage(width=16, height=16)
    
    def setup_speech_recognition(self):
        self.recognizer = sr.Recognizer()
        self.listening = False
        self.listen_thread = None
        
        try:
            self.microphone = sr.Microphone()
            with self.microphone as source:
                rospy.loginfo("Calibrating microphone for ambient noise...")
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                rospy.loginfo(f"Energy threshold set to: {self.recognizer.energy_threshold}")
        except Exception as e:
            rospy.logerr(f"Microphone setup failed: {e}")
            self.microphone = None
    
    def toggle_sil(self):
        self.sil_enabled = not self.sil_enabled
        self.sil_toggle_button.config(
            text="SIL ON" if self.sil_enabled else "SIL OFF",
            bg="#FF9800" if self.sil_enabled else "#9E9E9E"
        )
        
        command = "toggle_sil"
        self.sil_command_publisher.publish(String(data=command))
        
        status_msg = f"SIL framework {'enabled' if self.sil_enabled else 'disabled'}"
        self.add_message("System", status_msg, "system", False)
        self.sil_panel.update_status(status_msg)
    
    def clear_chat(self):
        self.chat_history.config(state=tk.NORMAL)
        self.chat_history.delete(1.0, tk.END)
        self.chat_history.config(state=tk.DISABLED)
        self.image_refs.clear()
        self.add_message("System", "Chat history cleared", "system", False)
    
    def recall_previous_message(self, event):
        if self.message_history:
            if self.history_index == -1:
                self.history_index = len(self.message_history) - 1
            elif self.history_index > 0:
                self.history_index -= 1
            
            if 0 <= self.history_index < len(self.message_history):
                self.user_input_entry.delete(0, tk.END)
                self.user_input_entry.insert(0, self.message_history[self.history_index])
        
        return "break" 
    
    def start_listening_thread(self):
        if self.tts_interface.is_speaking:
            self.add_message("System", "Please wait, I'm still speaking.", "system", False)
            return      
        
        if not self.listening:
            self.listening = True
            self.listen_button.config(text="Stop", bg="#FF5722")
            self.add_message("System", "🎤 Listening... Speak now", "system", False)
            self.listen_thread = threading.Thread(target=self.listen_and_transcribe)
            self.listen_thread.daemon = True
            self.listen_thread.start()
        else:
            self.stop_listening()
    
    def stop_listening(self):
        self.listening = False
        self.listen_button.config(text="Listen", bg="#2196F3")
    
    def listen_and_transcribe(self):
        try:
            if not self.microphone:
                self.after(0, self.add_message, "System", "No microphone detected", "system", False)
                return
            
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                rospy.loginfo(f"Adjusted energy threshold: {self.recognizer.energy_threshold}")
                
                try:
                    self.after(0, self.add_message, "System", "Listening... Speak clearly", "system", False)
                    
                    audio = self.recognizer.listen(
                        source,
                        timeout=10,
                        phrase_time_limit=15
                    )
                    
                    rospy.loginfo("Audio captured, recognizing...")
                    self.process_audio(audio)
                    
                except sr.WaitTimeoutError:
                    self.after(0, self.add_message, "System", "No speech detected. Try speaking louder.", "system", False)
                except sr.UnknownValueError:
                    self.after(0, self.add_message, "System", "Could not understand audio.", "system", False)
                except sr.RequestError as e:
                    rospy.logerr(f"Speech recognition API error: {e}")
                    self.after(0, self.add_message, "System", f"Speech service error: {e}", "system", False)
                    
        except Exception as e:
            rospy.logerr(f"Error in voice recognition thread: {e}")
            self.after(0, self.add_message, "System", f"Voice system error: {e}", "system", False)
        finally:
            self.after(0, self.stop_listening)
    
    def process_audio(self, audio):
        try:
            if rospy.get_param("debug/save_audio", False):
                try:
                    with open("/tmp/debug_audio.wav", "wb") as f:
                        f.write(audio.get_wav_data())
                    rospy.loginfo("Saved debug audio to /tmp/debug_audio.wav")
                except Exception as e:
                    rospy.logwarn(f"Could not save debug audio: {e}")
            
            result = self.recognizer.recognize_google(audio, show_all=True)
            rospy.loginfo(f"Google SR result: {result}")
            
            transcript = None
            confidence = 0.0
            
            if isinstance(result, dict) and 'alternative' in result and result['alternative']:
                best_result = result['alternative'][0]
                transcript = best_result.get('transcript', '')
                confidence = best_result.get('confidence', 0.0)
            elif isinstance(result, str):
                transcript = result
                confidence = 0.8  
            
            if transcript:
                rospy.loginfo(f"Recognized: '{transcript}' (confidence: {confidence:.2f})")
                self.publish_user_input(transcript)
                
                conf_text = f"Confidence: {confidence:.0%}" if confidence > 0 else ""
                if conf_text:
                    self.after(0, self.add_message, "System", conf_text, "system", False)
            else:
                self.after(0, self.add_message, "System", "No valid transcript found", "system", False)
                
        except sr.UnknownValueError:
            self.after(0, self.add_message, "System", "Could not understand audio", "system", False)
        except sr.RequestError as e:
            rospy.logerr(f"Speech recognition API error: {e}")
            self.after(0, self.add_message, "System", f"Speech service error: {e}", "system", False)
        except Exception as e:
            rospy.logerr(f"Audio processing error: {e}")
            self.after(0, self.add_message, "System", f"Audio processing error: {e}", "system", False)
    
    def publish_user_input(self, text):
        self.llm_input_publisher.publish(String(data=text))
        self.add_message("You", text, "user_msg", True)
        
        self.message_history.append(text)
        if len(self.message_history) > 20: 
            self.message_history = self.message_history[-20:]
        self.history_index = -1
    
    def send_message(self, event=None):
        user_input = self.user_input_entry.get().strip()
        if user_input:
            self.publish_user_input(user_input)
            self.user_input_entry.delete(0, tk.END)
    
    def add_llm_response(self, data):
        response = data.data
        self.after(0, self.add_message, "SIL_Robot", response, "bot_msg", False)
    
    def add_sil_response(self, data):
        response = data.data
        self.after(0, self.add_message, "SIL-Robot", response, "bot_msg", False)
    
    def add_clarification(self, data):
        clarification = data.data
        self.after(0, self.add_message, "SIL-Robot", f"{clarification}", "clarification", False)
    
    def add_suggestions(self, data):
        suggestions = data.data
        suggestions = data.data
        self.after(0, self.add_message, "SIL-Robot", f"{suggestions}", "suggestion", False)
    
    def update_sil_status(self, data):
        try:
            status_data = json.loads(data.data)
            if 'last_result' in status_data:
                self.current_interaction_id = f"interaction_{int(time.time())}"
                self.sil_panel.set_interaction_id(self.current_interaction_id)
            if 'performance_stats' in status_data:
                perf = status_data['performance_stats']
                if isinstance(perf, dict) and 'success_rate' in perf:
                    success_rate = perf.get('success_rate', 0.0)
                    clarity_rate = 1.0 - perf.get('clarification_rate', 0.0)
                    status_msg = f"Success: {success_rate:.0%}, Clarity: {clarity_rate:.0%}"
                    self.sil_panel.update_status(status_msg)
            
        except json.JSONDecodeError:
            rospy.logwarn("Invalid JSON in SIL status message")
        except Exception as e:
            rospy.logerr(f"Error processing SIL status: {e}")
    
    def add_llm_image(self, img_msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
            max_size = (400, 300)
            pil_img.thumbnail(max_size, PILImage.Resampling.LANCZOS)
            
            tk_img = ImageTk.PhotoImage(pil_img)
            self.image_refs.append(tk_img)
            
            self.after(0, self.add_image_to_chat, tk_img)
            
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            self.after(0, self.add_message, "System", "Failed to display image", "system", False)
        except Exception as e:
            rospy.logerr(f"Unexpected error in image display: {e}")
    
    def add_image_to_chat(self, tk_img):
        self.chat_history.config(state=tk.NORMAL)
        self.chat_history.insert(tk.END, "\n📷 Robot's View:\n")
        self.chat_history.image_create(tk.END, image=tk_img)
        self.chat_history.insert(tk.END, "\n\n")
        self.chat_history.config(state=tk.DISABLED)
        self.chat_history.see(tk.END)
    
    def add_message(self, sender, message, tag, is_user):
        icon = self.you_icon if is_user else self.llm_icon
        
        self.chat_history.config(state=tk.NORMAL)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        if icon and icon.width() > 1:  
            self.chat_history.image_create(tk.END, image=icon)
            self.chat_history.insert(tk.END, " ")

        formatted = f"[{timestamp}] {sender}: {message}\n"
        self.chat_history.insert(tk.END, formatted, ("sender", tag))
        self.chat_history.insert(tk.END, "\n")
        
        self.chat_history.config(state=tk.DISABLED)
        self.chat_history.see(tk.END)

        self.interaction_history.append({
            'timestamp': timestamp,
            'sender': sender,
            'message': message,
            'is_user': is_user
        })
        if len(self.interaction_history) > 100:
            self.interaction_history = self.interaction_history[-100:]
    
    def on_closing(self):
        rospy.loginfo("[SIL-GUI] Shutting down Chat Interface")
        self.listening = False
        if self.listen_thread and self.listen_thread.is_alive():
            self.listen_thread.join(timeout=1.0)
        try:
            self.sil_command_publisher.publish(String(data="gui_shutdown"))
        except:
            pass
        
        self.destroy()

def main():
    try:
        chat_interface = EnhancedChatInterface()
        chat_interface.protocol("WM_DELETE_WINDOW", chat_interface.on_closing)
        welcome_msg = ("SIL - Robot Ready!\n"
                      "Features: Voice recognition, proactive suggestions, clarification requests,\n"
                      "adaptive learning, and bidirectional interaction.\n"
                      "Try saying 'Hello' or ask me to move around!")
        chat_interface.add_message("", welcome_msg, "system", False)
        
        chat_interface.mainloop()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Chat GUI")
    except Exception as e:
        rospy.logerr(f"Fatal error in Chat GUI: {e}")
        raise

if __name__ == '__main__':
    main()
