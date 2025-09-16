
namespace APM_Running_App
{
    public partial class PatternElitePage : ContentPage
    {
        public int[] Distances { get; set; }
        public double[] Speeds { get; set; }

        public PatternElitePage()
        {
            InitializeComponent();

        }

        async void Button_Clicked(object sender, EventArgs e)
        {

            if (picker_dist.SelectedIndex == -1 || picker_sex.SelectedIndex == -1 || entry_time1.Text == "" || entry_time2.Text == "")
            {
                await DisplayAlert("Alert", "Please fill in all entries", "OK");
            }
            else
            {
                var response = await DisplayAlert("Start Run", "You have selected to start a run, do you want to continue?", "OK", "Cancel");
                if (response == true)
                {
                    double min = Convert.ToDouble(entry_time1.Text);
                    double sec = Convert.ToDouble(entry_time2.Text);
                    double finalTime_sec = sec + min*60;

                    var selectedDist = Convert.ToInt32(picker_dist.SelectedItem);
                    string selectedSex = Convert.ToString(picker_sex.SelectedItem);
                    var paceStrategy = GetElitePaceStrategy(selectedSex, selectedDist);
                    double[] speeds = paceStrategy.Speeds;
                    int[] distances = paceStrategy.Distances;
                    double[] optimalSpeedArray = CalculateOptimalSpeedPattern(distances, speeds, selectedDist, finalTime_sec);

                    await Navigation.PushAsync(new RunStartedPage(selectedDist.ToString(), min.ToString(), sec.ToString()));
                }
            }

        }
        void OnEntryTextChanged(object sender, TextChangedEventArgs e)
        {
            string oldText = e.OldTextValue;
            string newText = e.NewTextValue;
            string min = entry_time1.Text;
            string sec = entry_time2.Text;
        }

        void OnEntryCompleted(object sender, EventArgs e)
        {
            string text = ((Entry)sender).Text;
        }

        static double[] CalculateOptimalSpeedPattern(int[] distanceList, double[] speedList, double distance, double t_f)
        {
            // Calculate average of optimal speed pattern
            double v_avg_elite = speedList.Average();
            int N = speedList.Length;

            // Find average to finish in a given time
            double v_avg_desired = distance / t_f;

            // Move graph to keep to desired pace
            double[] optimalSpeedArray = new double[N];
            double change_speed = Math.Abs(v_avg_desired - v_avg_elite);
            optimalSpeedArray[0] = 0;

            for (int i = 1; i < N; i++)
            {
                double optimalSpeed = speedList[i] - change_speed;
                optimalSpeedArray[i] = optimalSpeed;
            }

            return optimalSpeedArray;
        }


        private PatternElitePage GetElitePaceStrategy(string selectedSex, int selectedDistance)
        {
            PatternElitePage result = new PatternElitePage();

            if (selectedSex == "Female")
            {
                if (selectedDistance == 800)
                {
                    result.Speeds = new double[] { 0, 7.44444445, 7.277777784, 6.750000005, 6.611111116, 6.638888894, 6.805555561, 6.638888894, 6.44444445 };
                    result.Distances = new int[] { 0, 100, 200, 300, 400, 500, 600, 700, 800 };
                }
                else if (selectedDistance == 1500)
                {
                    result.Speeds = new double[] { 0, 6.083333338, 5.694444449, 5.638888893, 5.611111116, 5.666666671, 5.55555556, 5.694444449, 5.861111116, 6.250000005, 6.388888894, 6.680555561, 6.750000005, 6.861111117, 6.527777783, 6.277777783 };
                    result.Distances = new int[] { 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500 };
                }
                else if (selectedDistance == 5000)
                {
                    result.Speeds = new double[] { 0, 5.277777782, 5.55555556, 5.569444449, 5.541666671, 5.638888893 };
                    result.Distances = new int[] { 0, 1000, 2000, 3000, 4000, 5000 };
                }
                else if (selectedDistance == 10000)
                {
                    result.Speeds = new double[] { 0, 5.250000004, 5.388888893, 5.375000004, 5.388888893, 5.416666671, 5.361111115, 5.361111115, 5.30555556, 5.277777782, 5.500000004 };
                    result.Distances = new int[] { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000 };
                }
            }
            else if (selectedSex == "Male")
            {
                // Handle male case
            }

            return result;
        }



    }

}
