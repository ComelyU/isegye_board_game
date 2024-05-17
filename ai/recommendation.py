import pandas as pd
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity


def comma_tokenizer(text):
    return [token.strip() for token in text.split(',') if token.strip()]


data = pd.read_csv('game.csv', encoding='CP949', low_memory=False)
data['combined'] = data['category'] + ',' + data['tag']

tfidf = TfidfVectorizer(tokenizer=comma_tokenizer, token_pattern=None)
tfidf_matrix = tfidf.fit_transform(data['combined'])
cosine_sim = cosine_similarity(tfidf_matrix, tfidf_matrix)

id_to_index = dict(zip(data['id'], data.index))
index_to_id = dict(zip(data.index, data['id']))


def get_recommendations(id, cosine_sim=cosine_sim):
    # 선택한 게임의 타이틀로부터 해당 게임의 인덱스를 받아온다.
    idx = id_to_index[id]

    # 해당 게임과 모든 게임와의 유사도를 가져온다.
    sim_scores = list(enumerate(cosine_sim[idx]))

    # 유사도에 따라 게임들을 정렬한다.
    sim_scores = sorted(sim_scores, key=lambda x: x[1], reverse=True)

    # 가장 유사한 3개의 게임을 받아온다.
    sim_scores = sim_scores[1:4]

    # 가장 유사한 3개의 게임의 인덱스를 얻는다.
    game_indices = [idx[0] for idx in sim_scores]

    # 가장 유사한 3개의 게임의 제목을 리턴한다.
    return [index_to_id[idx] for idx in game_indices]


def get_list():
    return data['id'].tolist()


if __name__ == '__main__':
    print(get_recommendations(11))


