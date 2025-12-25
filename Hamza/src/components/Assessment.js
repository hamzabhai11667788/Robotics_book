import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './Assessment.module.css';

const Assessment = ({ title, questions }) => {
  const [answers, setAnswers] = useState({});
  const [submitted, setSubmitted] = useState(false);

  const handleAnswerChange = (questionIndex, option) => {
    if (!submitted) {
      setAnswers({
        ...answers,
        [questionIndex]: option,
      });
    }
  };

  const handleSubmit = () => {
    setSubmitted(true);
  };

  const calculateScore = () => {
    let correct = 0;
    questions.forEach((question, index) => {
      if (answers[index] === question.correctAnswer) {
        correct++;
      }
    });
    return Math.round((correct / questions.length) * 100);
  };

  return (
    <div className={styles.assessmentContainer}>
      <h3>{title}</h3>
      {questions.map((question, qIndex) => (
        <div key={qIndex} className={styles.question}>
          <p><strong>Question {qIndex + 1}:</strong> {question.text}</p>
          <ul className={styles.options}>
            {question.options.map((option, oIndex) => (
              <li key={oIndex}>
                <label>
                  <input
                    type="radio"
                    name={`question-${qIndex}`}
                    value={oIndex}
                    checked={answers[qIndex] === oIndex}
                    disabled={submitted}
                    onChange={() => handleAnswerChange(qIndex, oIndex)}
                  />
                  <span>{option}</span>
                </label>
              </li>
            ))}
          </ul>
          {submitted && (
            <div className={styles.feedback}>
              {answers[qIndex] === question.correctAnswer ? (
                <span className={styles.correct}>✓ Correct!</span>
              ) : (
                <span className={styles.incorrect}>
                  ✗ Incorrect. Correct answer: {question.options[question.correctAnswer]}
                </span>
              )}
            </div>
          )}
        </div>
      ))}
      {!submitted && (
        <button className={styles.submitButton} onClick={handleSubmit}>
          Submit Answers
        </button>
      )}
      {submitted && (
        <div className={styles.score}>
          <p>Score: {calculateScore()}%</p>
        </div>
      )}
    </div>
  );
};

export default Assessment;