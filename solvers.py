from operator import index
import random

class SentenceCorrector(object):
    def __init__(self, cost_fn, conf_matrix):
        self.conf_matrix = conf_matrix
        self.cost_fn = cost_fn

        # You should keep updating following variable with best string so far.
        self.initi=0
        self.best_state = None  
        self.frontier = None
        self.replace = None

        self.store_vals = []

    def search(self, start_state):
        """
        :param start_state: str Input string with spelling errors
        """

        if self.initi==0:
            self.best_state = start_state
            self.initi=1

        self.dictr() #"invert" conf_matrix
        self.init_store() #initialise storage of ordered costs and variations

        words = self.best_state.split(" ")

        maxlen=0 
        for word in words:
            if len(word)>maxlen:
                maxlen = len(word)
        #maxlen is the maximum length of a word in this string

        self.best_state = self.cont_sweep(1)
        self.best_state = self.cont_sweep(2)

        for num_iters in range(0,3):
            for j in range(0, maxlen):
                better_state = self.change_words_w_storage(j)
                self.best_state = better_state

        
        string_spaced_sweep = self.spaced_sweep() #correcting with band of 3, non-space characters
        self.best_state = string_spaced_sweep #updating self.best_state

        self.word_1_char()
        self.word_2_char_wholes()

        self.best_state = self.cont_sweep(1)
        self.best_state = self.cont_sweep(2)
        self.best_state = self.cont_sweep(3)
        self.best_state = self.cont_sweep(4)


        #self.best_state = self.words_strat(self.best_state)
        #strings_cont_sweep = self.cont_sweeper() #correcting with bands of 1,2,3 across the length

        
        return self.best_state
        
        # You should keep updating self.best_state with best string so far.
        # self.best_state = start_state
        #raise Exception("Not Implemented.")

    def genPerm(self, strin , ind): #generates Permutations at index ind
            out = [strin]
            if strin[ind] == " ":
                return out
            inc_chars = self.replace[strin[ind]]
            for j in range(0, len(inc_chars)):
                out += [strin[:ind] + inc_chars[j] +strin[ind+1:]]
            return out

    def arrtarr(self, arr): #transforms array of array to array
        n = len(arr)
        out= []
        for i in range(0,n):
            ni= len(arr[i])
            for j in range(0,ni):
                out+= [arr[i][j]]
        return out

    def dictr(self): #inversing the conf matrix
        alpha = ["a","b","c","d","e","f","g","h","i","j","k","l","m","n","o","p","q","r","s","t","u","v","w","x","y","z"]
        
        newdict = {}
        [newdict.setdefault(x, []) for x in alpha] #initialising Dictionary
        
        for th in alpha:
            incexas = self.conf_matrix[th]
            for ince in incexas:
                l=0
                b = newdict[ince]
                if (th in b):
                    l=1
                else:
                    b += [th]
                    newdict.update({ince: b})
        self.replace=newdict

    def init_store(self): #matrix of tuples. [i][j]([0],[1])-> ith word, jth priority, (cost_func of sentence w word, word)
        words = self.best_state.split(" ")
        b = self.cost_fn(self.best_state)
        k = len(words)
        for j in range(0,k):
            self.store_vals += [[[b , words[j]]]]

    def add_to_store(self, storage, word, i, words): 
        if len(storage) < 100:
            k = len(storage)
            j = 0
            words[i] = word

            s=""
            for j in range(0, len(words)):
                s+= words[j] + " "

            s= s[:len(self.best_state)]
            c = self.cost_fn(s)
            insert_after = self.insert_in_ordered_after_index(storage, 0, k-1, c)

            if insert_after[1] == 1: # only insert if not already existant
                mid = insert_after[0] +1
                storage = storage[:mid] +[[self.cost_fn(s), word]] + storage[mid:]
                return storage
        return storage

    def insert_in_ordered_after_index(self, stori, l, r, val_to_be_inserted): #will indicate index in self.store_vals[i] where val (if not already present)is to be inserted
        if (r >= l):
            mid=l +((r - l) //2) #avoids overflow
            if (stori[mid][0] == val_to_be_inserted):
                return [mid, 0] #if value is found in the storage, we return 0 as the second element of the tuple
            elif stori[mid][0] > val_to_be_inserted: #check left
                return self.insert_in_ordered_after_index(stori, l, mid-1, val_to_be_inserted)
            else: #check right
                return self.insert_in_ordered_after_index(stori, mid + 1, r, val_to_be_inserted)
        else:
            return [r,1]

    def change_words_w_storage(self, layer): #main algorithm block
        words = self.best_state.split(" ")

        for word_i in range(0, len(words)): #ith word
            tem_stor_for_word_i = self.store_vals[word_i] #ith word ke current variations
            if layer < len(words[word_i]): #word len > layer
                for word_iter in range(0, len(self.store_vals[word_i])): #existing words in tem_stor_for_word_i of word i
                    word = self.store_vals[word_i][word_iter][1] #word_iter th word in tem_stor_for_word_i
                    new_words= self.genPerm(word, layer)#create variations of word at char number = layer
                    for t in new_words:
                        tem_stor_for_word_i=self.add_to_store(tem_stor_for_word_i, t, word_i, words) # adding new variations for that word, at char word_iter
                self.store_vals[word_i] = tem_stor_for_word_i
                words[word_i] = self.store_vals[word_i][0][1] #updating best of previous to be used in evaluation of next word's cost

        words_str = ""
        for word_i in range(0, len(words)):
            words_str += self.store_vals[word_i][0][1] + " "
        words_str = words_str[:len(self.best_state)]
        return words_str




#Refinement procedure
    def last_perm(self, k, s): #Last valid k non-space characters of string s
        n = len(s)
        if  k == 0:
            return []
        elif s[-1] != " ":
            return self.last_perm(k-1,s[:n-1]) + [n-1]
        else:
            return self.last_perm(k,s[:n-1])
    
    def first_perm(self, k, s): #First valid k non-space characters of string s
        n = len(s)
        it = 0
        out = []
        siz = 0
        while it < n:
            if  siz == k:
                return out
            elif s[it] != " ":
                out += [it]
                it += 1
                siz += 1
            else:
                it += 1

    def indices(self, curr, s): #next set of valid non-space characters
        n = len(curr)

        if len(curr) == 0:
            curr=[]
        elif curr[-1]!= len(s)-1: #curr ka last is not the last of string       
            if s[curr[-1]+1]!= " ": #not reached the eo s, and naya char is not empty
                g= curr[-1]
                curr.pop(0)
                curr += [g+1]
            elif (curr[-1]+2 == len(s)):
                print(0)
                curr= self.indices(curr[:n-1],s[:len(s)-2]) +[curr[-1]]
                print(curr[:n-1])
                print(s[:len(s)-2])
            else: #naya char is empty
                g= curr[-1]
                curr.pop(0)
                curr+= [g+2]
        else:      
            curr = self.indices(curr[:n-1], s[:len(s)-1]) + [curr[-1]]
        return curr

    def best_from_these_k(self, curr, s): # best sentence out of iterations in indices at curr 
        
        start=[s]
        for i in curr:
            new= []
            for j in range(0, len(start)):
                new += [self.genPerm(start[j], i)]
            start = self.arrtarr(new)
        arr4 = list(map(self.cost_fn, start))
        index_min = min(range(len(arr4)), key=arr4.__getitem__)
        return start[index_min]

    def spaced_sweep(self): #exclude space, band size 3, continuous space
        curr= self.first_perm(3, self.best_state) #first 3 valied iter points
        lass= self.last_perm(3, self.best_state) #last 3 valied iter points
        n = len(self.best_state)
        w = self.best_state
        while(curr != lass):
            w = self.best_from_these_k(curr, w)
            curr = self.indices(curr, self.best_state)
        return w


#Refining Algorithms
    def cont_sweep(self, k): #include space, sweep band, size k
        n = len(self.best_state)
        start_ind = [i for i in range(0,k)]
        end_ind = [n-k+i for i in range(0,k)]

        boo = True
        best_states =[]
        while boo:
            if (start_ind == end_ind):
                boo = False
            start=[self.best_state]

            for i in start_ind:
                new= []
                for j in range(0, len(start)):
                    new += [self.genPerm(start[j], i)]
                start = self.arrtarr(new)

            arr4 = list(map(self.cost_fn, start))
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            best_states += [start[index_min]]
            self.best_state = start[index_min]

            a = start_ind.pop(0)
            start_ind += [a+k]
        
        arr5 = list(map(self.cost_fn, best_states))
        index_min = arr5.index(min(arr5))
        self.best_state = best_states[index_min]
        return best_states[index_min]

    def cont_sweeper(self): #best cont sweep, size 1 ,2 or 3
        strings = [self.cont_sweep(1), self.cont_sweep(2), self.cont_sweep(3)]
        c = list(map(self.cost_fn, strings))
        cmin = min(c)
        ind= c.index(cmin)

        return strings[ind]

    

    def iter_word(self, word): #changing only 1 character, best option

        k = len(word)
        if k == 1:
            return word
        else:
            start=[]
            for i in range(0, k):
                start += self.genPerm(word, i)
            arr4 = list(map(self.cost_fn, start)) #finding cost functiond of iterations
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            return start[index_min]

    def iter_word_whole_cost_eval(self, word, words, i): #changing only 1 character, best option

        k = len(word)
        if k == 1:
            return word
        else:
            start=[]
            sents = []
            sleft = ""
            sright = ""
            for ff in range(0,i):
                sleft += words[ff] + " "
            if i != len(words)-1:
                for ff in range(i+1, len(words)):
                    sright += words[ff] + " "

            for gh in range(0, k):
                start += self.genPerm(word, gh)

            for ss in start:
                sent = sleft + ss + " " + sright
                sent = sent[:len(self.best_state)]
                sents += [sent]
                
            arr4 = list(map(self.cost_fn, sents)) #finding cost functiond of iterations
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            return start[index_min]
  
    def iter_word_2(self, word): #band sweep across word

        k = len(word)

        if k == 1:
            return word
        elif k==2:
            start=[word]
            for i in range(0, 2):
                new= []
                for j in range(0, len(start)):
                    new += [self.genPerm(start[j], i)]
                start = self.arrtarr(new)

            arr4 = list(map(self.cost_fn, start))
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            return start[index_min]
        else:
            start_ind = [0,1]
            end_ind = [k-2,k-1]
            boo = True
            while boo:
                if (start_ind == end_ind):
                    boo = False
                start=[word]
                for i in start_ind:
                    new= []
                    for j in range(0, len(start)):
                        new += [self.genPerm(start[j], i)]
                    start = self.arrtarr(new)

                arr4 = list(map(self.cost_fn, start))
                index_min = min(range(len(arr4)), key=arr4.__getitem__)
                word = start[index_min]

                a = start_ind.pop(0)
                start_ind += [a+2]

            return word
        
    def iter_word_2_whole_cost_eval(self, word, words, i): #band sweep across word

        k = len(word)

        if k == 1:
            return word
        elif k==2:
            start=[word]
            sents = []
            sleft = ""
            sright = ""

            for ff in range(0,i):
                sleft += words[ff] + " "
            if i != len(words)-1:
                for ff in range(i+1, len(words)):
                    sright += words[ff] + " "
                
            for i in range(0, 2):
                new= []
                for j in range(0, len(start)):
                    new += [self.genPerm(start[j], i)]
                start = self.arrtarr(new)

            for ss in start:
                sent = sleft + ss + " " + sright
                sent = sent[:len(self.best_state)]
                sents += [sent]

            arr4 = list(map(self.cost_fn, sents))
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            return start[index_min]
        else:
            start_ind = [0,1]
            end_ind = [k-2,k-1]

            sents = []
            sleft = ""
            sright = ""

            for ff in range(0,i):
                sleft += words[ff] + " "
            if i != len(words)-1:
                for ff in range(i+1, len(words)):
                    sright += words[ff] + " "

            boo = True
            start=[word]
            starter =[]
            while boo:
                if (start_ind == end_ind):
                    boo = False
                for sind in start_ind:
                    new= []
                    for j in range(0, len(start)):
                        new += [self.genPerm(start[j], sind)]
                    start = self.arrtarr(new)

                    sebts = []
                    for ss in start:
                        sent = sleft + ss + " " + sright
                        sent = sent[:len(self.best_state)]
                        sebts += [sent]
                    
                    arr4 = list(map(self.cost_fn, sebts))
                    index_min = min(range(len(arr4)), key=arr4.__getitem__)
                    start = [start[index_min]]
                    starter+= start
                    

                a = start_ind.pop(0)
                start_ind += [a+2]
            
            for ss in starter:
                sent = sleft + ss + " " + sright
                sent = sent[:len(self.best_state)]
                sents += [sent]

            arr4 = list(map(self.cost_fn, sents))
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            word = starter[index_min]
            self.best_state = sents[index_min]
            return word

    def word_1_char(self):
        words = self.best_state.split(" ")
        new_string=""
        for ind in range(0, len(words)):
            word = words[ind]
            new_string += self.iter_word_whole_cost_eval(word, words, ind) + " "
        self.best_state = new_string[:len(self.best_state)]
        return new_string[:len(self.best_state)]

    def word_2_char_wholes(self):
        words = self.best_state.split(" ")
        new_string=""
        for ind in range(0, len(words)):
            word = words[ind]
            new_string += self.iter_word_2_whole_cost_eval(word, words, ind) + " "
        self.best_state = new_string[:len(self.best_state)]
        return new_string[:len(self.best_state)]


    def words_at_a_time(self, s, words, curr): #3 words, random iterations, pick best
        best_states =[]
        best_words = []
        for k in range(0,100):

            n = len(s)
            start_ind = random.sample(range(0,n), 3) 
            start=[s]
            for i in start_ind:
                new= []
                for j in range(0, len(start)):
                    new += [self.genPerm(start[j], i)]
                start = self.arrtarr(new)

            sents = []
            for sent in start:
                string = ""
                for j in range(0, len(words)):
                    if (j!=curr[0] and j!=curr[1] and j!=curr[2]):
                        string += words[j] + " "
                    elif j == curr[0]:
                        string += sent + " "
                string = string[:len(self.best_state)]
                sents += [string]

            arr4 = list(map(self.cost_fn, sents))
            index_min = min(range(len(arr4)), key=arr4.__getitem__)
            best_states += [sents[index_min]]
            best_words += [start[index_min]]

            
        arr5 = list(map(self.cost_fn, best_states))
        index_min = arr5.index(min(arr5))
            
        return best_words[index_min]

    def words_strat(self, s): #strategy picks 3 words at a time, band sweeps w time
        words = s.split(" ")
        k = len(words)

        start_ind = [0,1,2]
        end_ind = [k-3,k-2,k-1]

        boo = True
        while boo:
            if(end_ind == start_ind):
                boo=False
            word_new = self.words_at_a_time(words[start_ind[0]] + " " + words[start_ind[1]]+ " " + words[start_ind[2]], words, start_ind)
            words_new = word_new.split(" ")
            words[start_ind[0]]= words_new[0]
            words[start_ind[1]]= words_new[1]
            words[start_ind[2]]= words_new[2]
            a=start_ind.pop(0)
            start_ind+= [a+3]

        string = ""
        for elem in range(0,k):
            string += words[elem] + " "
        string = string[:len(self.best_state)]

        return string

    
        

                