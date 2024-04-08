import numpy as np

class MultinomialNaiveBayes:
    def __init__(self, alpha=1):
        self.alpha = alpha  # Laplace smoothing parameter
        self.class_prior_ = None
        self.feature_prob_ = None

    def fit(self, X, y):
        self.classes_ = np.unique(y)
        n_classes = len(self.classes_)
        n_features = X.shape[1]

        # Calculate class prior probabilities
        self.class_prior_ = np.zeros(n_classes)
        for i, c in enumerate(self.classes_):
            self.class_prior_[i] = np.sum(y == c) / float(len(y))

        # Calculate smoothed feature probabilities
        self.feature_prob_ = np.zeros((n_classes, n_features))
        for i, c in enumerate(self.classes_):
            class_instances = X[y == c]
            total_count = np.sum(class_instances) + n_features * self.alpha
            self.feature_prob_[i] = (np.sum(class_instances, axis=0) + self.alpha) / total_count

    def predict_proba(self, X):
        return np.exp(self.predict_log_proba(X))

    def predict_log_proba(self, X):
        n_classes = len(self.classes_)
        n_samples, n_features = X.shape
        log_prob = np.zeros((n_samples, n_classes))

        for i in range(n_samples):
            for j in range(n_classes):
                log_prob[i, j] = np.sum(np.log(self.feature_prob_[j]) * X[i])

        log_prob += np.log(self.class_prior_)
        return log_prob


    def predict(self, X):
        return self.classes_[np.argmax(self.predict_log_proba(X), axis=1)]